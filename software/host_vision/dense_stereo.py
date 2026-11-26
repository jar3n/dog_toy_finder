"""Dense-stereo depth backends for the host-side ``stereo-net`` pipeline.

Runs only on the computer (see ``stereo_service.py``). The ROS container sends
one rectified JPEG pair plus one sampling request; this module runs the selected
dense model and returns the median disparity (and derived depth) at the sampled
pixels. Torch, the three upstream repositories, and their weights never enter
the ROS container.

One model runs at a time, selected with ``DEPTH_MODEL`` or ``--model``:

``lite_any_stereo``
    Lite Any Stereo V2, small checkpoint (``LAS2_S.pth``).
``fast_foundation_stereo``
    NVIDIA Fast-FoundationStereo 23-36-37 checkpoint.
``rt_igevpp``
    IGEV++ real-time configuration (six GRU iterations, ``core_rt`` network).

All weights are resolved from local files prepared by
``scripts/start_stereo_service.sh``. That script uses HTTPS git checkouts and
Hugging Face only; it never downloads from a file-sharing drive. Heavy imports
are lazy so ``--help``, health checks, and tests need no torch or checkpoints.
Tests can inject a fake model with ``load_model(config, model=fake)``.
"""

import contextlib
import os
import sys
from pathlib import Path
from types import SimpleNamespace

import numpy as np

HOST_VISION = Path(__file__).resolve().parent
MODELS_DIR = HOST_VISION / "models"

LITE_ANY_STEREO = "lite_any_stereo"
FAST_FOUNDATION_STEREO = "fast_foundation_stereo"
RT_IGEVPP = "rt_igevpp"
MODELS = (LITE_ANY_STEREO, FAST_FOUNDATION_STEREO, RT_IGEVPP)
DEFAULT_MODEL = LITE_ANY_STEREO

# Relative paths under software/host_vision/models. The setup script downloads
# these from Hugging Face repositories; no service runs an ad-hoc download.
WEIGHTS_PATHS = {
    LITE_ANY_STEREO: "LAS2_S.pth",
    FAST_FOUNDATION_STEREO: "fast_foundation_stereo/model_best_bp2_serialize.pth",
    RT_IGEVPP: "rt_igevpp/sceneflow.pth",
}
WEIGHTS_ENV = {
    LITE_ANY_STEREO: "LITE_ANY_STEREO_MODEL_PATH",
    FAST_FOUNDATION_STEREO: "FAST_FOUNDATION_STEREO_MODEL_PATH",
    RT_IGEVPP: "RT_IGEVPP_MODEL_PATH",
}
REPO_ENV = {
    LITE_ANY_STEREO: "LITE_ANY_STEREO_REPO",
    FAST_FOUNDATION_STEREO: "FAST_FOUNDATION_STEREO_REPO",
    RT_IGEVPP: "IGEV_PLUSPLUS_REPO",
}
REPO_PATHS = {
    LITE_ANY_STEREO: "repos/LiteAnyStereo",
    FAST_FOUNDATION_STEREO: "repos/Fast-FoundationStereo",
    RT_IGEVPP: "repos/IGEV-plusplus",
}
DEFAULT_ITERS = {
    LITE_ANY_STEREO: 1,          # the network predicts a dense map in one pass
    FAST_FOUNDATION_STEREO: 8,
    RT_IGEVPP: 6,                # the real-time IGEV++ setting
}

_CACHE = {}


def model_name(config):
    """Return the selected dense-stereo backend name from config or env."""
    if isinstance(config, str):
        raw = config
    else:
        raw = (config or {}).get("model") or (config or {}).get("name")
    raw = raw or os.environ.get("DEPTH_MODEL") or DEFAULT_MODEL
    name = str(raw).strip().lower().replace("-", "_")
    if name not in MODELS:
        raise ValueError("Unknown dense stereo model %r; choose one of: %s"
                         % (name, ", ".join(MODELS)))
    return name


def default_weights(name):
    """Return the local checkpoint path for one backend."""
    name = model_name(name)
    configured = os.environ.get(WEIGHTS_ENV[name])
    return configured or str(MODELS_DIR / WEIGHTS_PATHS[name])


def _repo_path(name, config=None):
    configured = (config or {}).get("repo_path") if isinstance(config, dict) else None
    configured = configured or os.environ.get(REPO_ENV[name])
    return Path(configured) if configured else MODELS_DIR / REPO_PATHS[name]


def _require_repo(name, config=None):
    repo = _repo_path(name, config).expanduser()
    if not repo.is_dir():
        raise FileNotFoundError(
            "Dense stereo source for %s not found at %s. Run "
            "./scripts/start_stereo_service.sh first (or set %s)."
            % (name, repo, REPO_ENV[name]))
    text = str(repo)
    if text not in sys.path:
        sys.path.insert(0, text)
    return repo


def _require_weights(path):
    path = Path(str(path)).expanduser()
    if not path.is_file() or path.stat().st_size == 0:
        raise FileNotFoundError(
            "Dense stereo checkpoint not found at %s. Run "
            "./scripts/start_stereo_service.sh (or set the model's weights "
            "environment variable) before starting the service." % path)
    return str(path)


def resolve_device(config=None):
    """Use the configured device, else CUDA when available, else CPU."""
    configured = (config or {}).get("device") or os.environ.get("STEREO_DEVICE")
    if configured:
        return str(configured)
    try:
        import torch
        return "cuda" if torch.cuda.is_available() else "cpu"
    except ImportError:
        return "cpu"


def inference_iters(name, config=None):
    """Return the number of update iterations for the selected backend."""
    configured = (config or {}).get("iters") if isinstance(config, dict) else None
    if configured in (None, ""):
        return DEFAULT_ITERS[model_name(name)]
    return int(configured)


def max_disparity(config=None):
    configured = (config or {}).get("max_disp") if isinstance(config, dict) else None
    return int(configured or 192)


def _torch_load(torch, path, device, weights_only=False):
    """Load a checkpoint with both old and new torch signatures."""
    try:
        return torch.load(path, map_location=device,
                          weights_only=weights_only)
    except TypeError:
        return torch.load(path, map_location=device)


def _timm_create_model_with_act1(torch, timm):
    """Return a create_model wrapper for newer timm feature extractors.

    IGEV++'s real-time feature network expects ``model.act1``. Recent timm
    versions fold that ReLU6 into ``model.bn1`` instead. Adding an identity
    module keeps the upstream forward pass valid without changing checkpoints.
    """
    original = timm.create_model

    def create_model(*args, **kwargs):
        feature_model = original(*args, **kwargs)
        if not hasattr(feature_model, "act1"):
            feature_model.act1 = torch.nn.Identity()
        return feature_model

    return create_model, original


def load_model(config, model=None):
    """Load and cache one dense stereo backend, importing torch lazily."""
    name = model_name(config)
    if model is not None:
        return name, model
    path = _require_weights((config or {}).get("model_path")
                            or default_weights(name))
    device = resolve_device(config)
    key = (name, path, device, max_disparity(config), inference_iters(name, config))
    if key in _CACHE:
        return name, _CACHE[key]

    try:
        import torch
    except ImportError as error:  # pragma: no cover - host venv dependency
        raise ImportError(
            "The host stereo service needs torch. Install it with "
            "scripts/start_stereo_service.sh") from error
    torch.autograd.set_grad_enabled(False)

    if name == LITE_ANY_STEREO:
        _require_repo(name, config)
        from core.models import build_model, load_model_weights
        version = str((config or {}).get("version") or "v2").strip().lower()
        size = str((config or {}).get("model_size") or "s").strip().lower()
        network = build_model(version, fnet_pretrained=False,
                              model_size=size, max_disp=max_disparity(config))
        checkpoint = _torch_load(torch, path, device)
        load_model_weights(network, checkpoint, strict=True)
    elif name == FAST_FOUNDATION_STEREO:
        # The checkpoint is a serialized nn.Module, so its repo must be on the
        # import path before torch.load unpickles it.
        _require_repo(name, config)
        network = _torch_load(torch, path, "cpu", weights_only=False)
        network.args.valid_iters = inference_iters(name, config)
        network.args.max_disp = max_disparity(config)
    else:
        _require_repo(name, config)
        import timm
        patched_create_model, original_create_model = (
            _timm_create_model_with_act1(torch, timm))
        timm.create_model = patched_create_model
        try:
            from core_rt.rt_igev_stereo import IGEVStereo
            args = SimpleNamespace(
                hidden_dim=96, corr_levels=2, corr_radius=4, n_downsample=2,
                n_gru_layers=3, max_disp=max_disparity(config),
                mixed_precision=False, precision_dtype="float32")
            network = IGEVStereo(args)
        finally:
            timm.create_model = original_create_model
        state = _torch_load(torch, path, "cpu")
        if isinstance(state, dict) and "state_dict" in state:
            state = state["state_dict"]
        clean = {key[7:] if key.startswith("module.") else key: value
                 for key, value in state.items()}
        network.load_state_dict(clean, strict=True)

    loaded = network.to(device).eval()
    _CACHE[key] = loaded
    return name, loaded


def _pad_pair(left, right, multiple=32):
    """Pad a rectified pair to the multiple required by the dense networks."""
    left = np.asarray(left, dtype=np.float32)
    right = np.asarray(right, dtype=np.float32)
    if left.shape != right.shape:
        raise ValueError("left and right images must have the same shape")
    height, width = left.shape[:2]
    pad_y = (-height) % multiple
    pad_x = (-width) % multiple
    padding = ((0, pad_y), (0, pad_x), (0, 0))
    return (np.pad(left, padding, mode="edge"),
            np.pad(right, padding, mode="edge"), (height, width))


def disparity_map(name, model, left_rgb, right_rgb, config=None):
    """Return a dense disparity array aligned with the unpadded left image."""
    name = model_name(name)
    try:
        import torch
    except ImportError as error:  # pragma: no cover - host venv dependency
        raise ImportError("Dense stereo inference needs torch") from error

    padded_left, padded_right, (height, width) = _pad_pair(left_rgb, right_rgb)
    device = resolve_device(config)
    tensors = [
        torch.as_tensor(np.ascontiguousarray(image)).float()[None].permute(
            0, 3, 1, 2).to(device)
        for image in (padded_left, padded_right)]

    with torch.no_grad():
        if name == LITE_ANY_STEREO:
            disparity = model(tensors[0], tensors[1],
                              max_disp=max_disparity(config), test_mode=True)
        elif name == FAST_FOUNDATION_STEREO:
            use_cuda = str(device).startswith("cuda") and torch.cuda.is_available()
            autocast = (torch.autocast("cuda", enabled=True) if use_cuda
                        else contextlib.nullcontext())
            with autocast:
                disparity = model.forward(
                    tensors[0], tensors[1],
                    iters=inference_iters(name, config), test_mode=True,
                    optimize_build_volume="pytorch1")
        else:
            disparity = model(tensors[0], tensors[1],
                              iters=inference_iters(name, config),
                              test_mode=True)

    if isinstance(disparity, (list, tuple)):
        disparity = disparity[-1]
    array = disparity.float().detach().cpu().numpy().reshape(
        padded_left.shape[0], padded_left.shape[1])
    return np.ascontiguousarray(array[:height, :width], dtype=np.float32)


def _as_point(value):
    if not isinstance(value, (list, tuple)) or len(value) < 2:
        raise ValueError("each sample point must be [u, v]")
    return float(value[0]), float(value[1])


def sample_disparity(disparity, points=None, window=None, config=None):
    """Return the median rectified disparity and derived depth over the samples.

    Dense networks report ``u_left - u_right``. When the rectified principal
    points differ, the container sends ``left.cx - right.cx`` as
    ``disparity_offset_px``; subtracting it gives the disparity compatible with
    ``distance = fx * baseline / disparity``.
    """
    camera = (config or {}).get("camera", {}) if isinstance(config, dict) else {}
    fx = float(camera.get("fx", 250.0))
    baseline = float(camera.get("baseline_m", 0.08))
    offset = float(camera.get("disparity_offset_px", 0.0))
    limit = max_disparity(config)
    array = np.asarray(disparity, dtype=float)
    height, width = array.shape[:2]

    if points:
        samples = []
        for point in points:
            u, v = _as_point(point)
            x = min(width - 1, max(0, int(round(u))))
            y = min(height - 1, max(0, int(round(v))))
            samples.append(array[y, x])
        source = "mask_pixels"
    elif window:
        u, v = _as_point((window.get("u"), window.get("v")))
        radius = int(window.get("radius", 3))
        x = min(width - 1, max(0, int(round(u))))
        y = min(height - 1, max(0, int(round(v))))
        x1, x2 = max(0, x - radius), min(width, x + radius + 1)
        y1, y2 = max(0, y - radius), min(height, y + radius + 1)
        samples = array[y1:y2, x1:x2].ravel()
        source = "box_window"
    else:
        raise ValueError("either 'points' or 'window' must name depth samples")

    samples = np.asarray(samples, dtype=float)
    corrected = samples - offset
    valid = np.isfinite(corrected) & (corrected > 0) & (corrected <= limit)
    result = {
        "sample_source": source,
        "sample_count": int(samples.size),
        "valid_count": int(valid.sum()),
        "disparity_offset_px": round(offset, 4),
        "raw_disparity_px": None,
        "disparity_px": None,
        "depth_m": None,
    }
    if not valid.any() or fx <= 0 or baseline <= 0:
        return result
    disparity_px = float(np.median(corrected[valid]))
    result["raw_disparity_px"] = round(float(np.median(samples[valid])), 4)
    result["disparity_px"] = round(disparity_px, 4)
    result["depth_m"] = round(fx * baseline / disparity_px, 4)
    return result
