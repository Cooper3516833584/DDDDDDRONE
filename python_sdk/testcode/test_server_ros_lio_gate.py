"""Test the mission topic gate without importing server_ros hardware startup."""

import ast
import math
import os
import re
from pathlib import Path

import pytest


source = Path(__file__).resolve().parents[1] / "server_ros.py"
tree = ast.parse(source.read_text(encoding="utf-8"))
gate = [node for node in tree.body
        if (isinstance(node, ast.Assign)
            and any(isinstance(target, ast.Name) and target.id == "REQUIRED_LIO_TOPICS"
                    for target in node.targets))
        or (isinstance(node, ast.FunctionDef) and node.name == "missing_lio_topics")]
namespace = {}
exec(compile(ast.fix_missing_locations(ast.Module(body=gate, type_ignores=[])),
             str(source), "exec"), namespace)


def test_missing_topics_are_reported_and_all_topics_pass():
    missing = namespace["missing_lio_topics"]
    assert missing(["/livox/lidar", "/Odometry"]) == ["/livox/imu", "/Odometry_highrate", "/LioHealth"]
    assert missing(namespace["REQUIRED_LIO_TOPICS"]) == []


def test_production_mount_gate_keeps_ros_diagnostics_available(tmp_path):
    config = tmp_path / "ros2_ws/src/FAST_LIO_ROS2/config/mid360s_drone.yaml"
    config.parent.mkdir(parents=True)
    config.write_text("extrinsic_est_en: false\nextrinsic_T: [0, 0, 0]\n"
                      "extrinsic_R: [1, 0, 0, 0, 1, 0, 0, 0, 1]\n", encoding="utf-8")
    sdk = tmp_path / "python_sdk"
    sdk.mkdir()
    function = next(node for node in tree.body
                    if isinstance(node, ast.FunctionDef)
                    and node.name == "require_production_localization")
    gate_namespace = {"os": os, "ast": ast, "math": math, "re": re, "PATH": str(sdk)}
    class FakeProvider:
        mount_reviewed = False

    gate_namespace["LioPoseProvider"] = FakeProvider
    exec(compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                 str(source), "exec"), gate_namespace)
    check = gate_namespace["require_production_localization"]
    check()
    with pytest.raises(RuntimeError, match="Reviewed MID360S body mount"):
        check(require_mount=True)
    FakeProvider.mount_reviewed = True
    check(require_mount=True)
