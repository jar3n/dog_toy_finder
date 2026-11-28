"""The stereo launch file must take camera geometry from the detector YAML."""

import importlib.util
import os

import yaml
from launch import LaunchContext
from launch_ros.actions import ComposableNodeContainer, Node

LAUNCH_FILE = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    "launch", "stereo_vision.launch.py")


def _load_module():
    spec = importlib.util.spec_from_file_location("stereo_vision_launch", LAUNCH_FILE)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _launch_context(config_path, overrides=None):
    values = dict(
        config_path=config_path,
        left_url="http://left/capture",
        right_url="http://right/capture",
        rate="2.0",
        width="", height="", baseline="", calibration_file="",
        left_fx="", left_fy="", left_cx="", left_cy="",
        right_fx="", right_fy="", right_cx="", right_cy="",
        stereo="true", stereo_proc="false", vision="false",
        approximate_sync="true", use_color="true", stereo_algorithm="0",
        min_disparity="0", disparity_range="64",
        pipeline="", yolo_service_url="", stereo_service_url="",
    )
    values.update(overrides or {})
    context = LaunchContext()
    for name, value in values.items():
        context.launch_configurations[name] = value
    return context


def _camera_params(module, config_path, overrides=None):
    context = _launch_context(config_path, overrides)
    group = module._launch_setup(context)[0]
    nodes = [entity for entity in group.get_sub_entities()
             if isinstance(entity, Node)
             and entity.node_executable == "stereo_camera_node"]
    assert len(nodes) == 1
    raw = nodes[0]._Node__parameters[0]
    return {list(key)[0].text: value for key, value in raw.items()}


def test_camera_geometry_defaults_come_from_yaml(tmp_path):
    config = {
        "camera": {
            "width": 640, "height": 480,
            "left": {"fx": 500.0, "fy": 510.0, "cx": 320.0, "cy": 240.0},
            "right": {"fx": 505.0, "fy": 515.0, "cx": 300.0, "cy": 238.0},
            "baseline_m": 0.123,
        },
    }
    path = tmp_path / "vision.yaml"
    path.write_text(yaml.safe_dump(config))

    params = _camera_params(_load_module(), str(path))

    assert params["width"] == 640
    assert params["height"] == 480
    assert params["left_fx"] == 500.0
    assert params["left_fy"] == 510.0
    assert params["left_cx"] == 320.0
    assert params["left_cy"] == 240.0
    assert params["right_fx"] == 505.0
    assert params["right_fy"] == 515.0
    assert params["right_cx"] == 300.0
    assert params["right_cy"] == 238.0
    assert params["baseline"] == 0.123
    assert params["calibration_file"] == ""


def test_calibration_file_is_forwarded_to_camera_node(tmp_path):
    path = tmp_path / "vision.yaml"
    path.write_text(yaml.safe_dump({"camera": {}}))

    params = _camera_params(
        _load_module(), str(path),
        {"calibration_file": "/tmp/stereo_calibration.json"})

    assert params["calibration_file"] == "/tmp/stereo_calibration.json"


def test_explicit_camera_args_override_yaml(tmp_path):
    path = tmp_path / "vision.yaml"
    path.write_text(yaml.safe_dump({
        "camera": {
            "left": {"fx": 500.0},
            "right": {"cx": 150.0},
            "baseline_m": 0.123,
        },
    }))

    params = _camera_params(
        _load_module(), str(path),
        {"left_fx": "999.5", "right_cx": "140.25", "baseline": "0.42"})

    assert params["left_fx"] == 999.5
    assert params["right_cx"] == 140.25
    assert params["baseline"] == 0.42


def test_stereo_image_proc_bool_params_are_booleans(tmp_path):
    path = tmp_path / "vision.yaml"
    path.write_text(yaml.safe_dump({"camera": {}}))

    module = _load_module()
    context = _launch_context(
        str(path), {"stereo_proc": "true", "approximate_sync": "true",
                    "use_color": "false"})
    group = module._launch_setup(context)[0]
    containers = [entity for entity in group.get_sub_entities()
                  if isinstance(entity, ComposableNodeContainer)]
    assert len(containers) == 1
    descriptions = containers[0]._ComposableNodeContainer__composable_node_descriptions
    params = []
    for node in descriptions:
        raw = node._ComposableNode__parameters[0]
        params.append({list(key)[0].text: value for key, value in raw.items()})

    assert [item["approximate_sync"] for item in params] == [True, True]
    assert params[0]["use_system_default_qos"] is False
    assert params[1]["use_color"] is False
