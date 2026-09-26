"""Test the mission topic gate without importing server_ros hardware startup."""

import ast
from pathlib import Path


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
    assert missing(["/livox/lidar", "/Odometry"]) == ["/livox/imu", "/Odometry_highrate"]
    assert missing(namespace["REQUIRED_LIO_TOPICS"]) == []
