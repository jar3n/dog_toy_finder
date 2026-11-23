"""Selectable vision pipelines over one rectified stereo pair.

Every pipeline takes a text target plus the rectified left/right JPEGs and
returns answer entries in the ``/vision/result`` schema. ``open_vocab`` uses an
open-vocabulary detector on each eye and triangulates matched detections;
``second`` is reserved for the pipeline described later.

The choice is resolved from, in order of precedence: an explicit value (launch
parameter or CLI flag), ``VISION_PIPELINE``, the detector YAML, then the default.
"""

import os

OPEN_VOCAB = "open_vocab"
SECOND = "second"
PIPELINES = (OPEN_VOCAB, SECOND)


def pipeline_name(explicit=None, config=None, environ=None):
    """Return a validated pipeline name from the first configured source."""
    env = os.environ if environ is None else environ
    name = (explicit or env.get("VISION_PIPELINE")
            or (config or {}).get("pipeline") or OPEN_VOCAB)
    name = str(name).strip().lower()
    if name not in PIPELINES:
        raise ValueError("Unknown vision pipeline %r; choose one of: %s"
                         % (name, ", ".join(PIPELINES)))
    return name


def run_pipeline(name, target, left_jpeg, right_jpeg, config, client=None):
    """Run one pipeline and return a list of answer entries.

    ``client`` is the detector callable injected by tests; production uses the
    HTTP client to the host-side detector service.
    """
    name = pipeline_name(name, config)
    if name == OPEN_VOCAB:
        from . import open_vocab_stereo
        return [open_vocab_stereo.detect(
            left_jpeg, right_jpeg, target, config, client=client)]
    raise NotImplementedError(
        "The 'second' vision pipeline is reserved but not configured yet. "
        "Use pipeline:=open_vocab until it is described.")
