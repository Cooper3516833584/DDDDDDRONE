"""Static FAST-LIO patch contracts; application and ROS2 build remain required."""

from pathlib import Path


PATCH = Path(__file__).resolve().parents[2] / "ros2_ws/patches/fast_lio_highrate.patch"


def mapping_additions():
    text = PATCH.read_text(encoding="utf-8")
    mapping = text.split("diff --git a/src/laserMapping.cpp b/src/laserMapping.cpp\n", 1)[1]
    mapping = mapping.split("\ndiff --git ", 1)[0]
    return "\n".join(line[1:] for line in mapping.splitlines()
                     if line.startswith("+") and not line.startswith("+++"))


def block_after(text, marker):
    start = text.index("{", text.index(marker) + len(marker))
    depth = 0
    for position in range(start, len(text)):
        if text[position] == "{":
            depth += 1
        elif text[position] == "}":
            depth -= 1
            if depth == 0:
                return text[start + 1:position], position + 1
    raise AssertionError("Unclosed patch block")


def test_current_scan_state_committed_before_highrate_reanchor():
    text = mapping_additions()
    markers = [
        "auto predicted_state = kf.get_x()",
        "auto predicted_covariance = kf.get_P()",
        "const bool laser_corrected = kf.update_iterated_dyn_share_modified",
        "kf.change_x(predicted_state)",
        "kf.change_P(predicted_covariance)",
        "state_point = kf.get_x()",
        "kf_fast.change_x(state_point)",
        "kf_fast.change_P(corrected_covariance)",
        "for (const auto &sample : highrate_imu_history)",
    ]
    positions = [text.index(marker) for marker in markers]
    assert positions == sorted(positions)
    assert text[positions[2]:positions[-1]].count("state_point = kf.get_x()") == 1


def test_rejected_iteration_restores_prediction_without_correction_or_early_return():
    rejected, _ = block_after(mapping_additions(), "if (!laser_corrected)")
    assert "kf.change_x(predicted_state)" in rejected
    assert "kf.change_P(predicted_covariance)" in rejected
    assert "correction_status = fast_lio::msg::LioHealth::DEGRADED" in rejected
    for forbidden in ("return;", "++correction_seq", "correction_anchor_ns =",
                      "kf_fast.change_", "map_incremental()"):
        assert forbidden not in rejected


def test_highrate_reanchor_requires_success_and_valid_only_anchor_is_preserved():
    reanchor, _ = block_after(mapping_additions(),
                             "if (laser_corrected && p_imu->imu_ready() && !Measures.imu.empty())")
    valid, _ = block_after(reanchor, "if (correction_valid)")
    assert "++correction_seq" in valid
    assert "correction_anchor_ns = anchor_ns" in valid
    assert reanchor.count("++correction_seq") == 1
    assert reanchor.count("correction_anchor_ns =") == 1
    assert "if (correction_seq == 0)" in reanchor
    assert "kf_fast.change_x(state_point)" in reanchor
    assert "auto corrected_covariance = kf.get_P()" in reanchor
    assert "<= anchor_ns) continue;" in reanchor


def test_rejected_frames_cannot_increment_map_and_clear_frame_statistics():
    text = mapping_additions()
    accepted, end = block_after(text, "if (laser_corrected)")
    assert "map_incremental();" in accepted
    assert text.count("map_incremental();") == 1
    assert text[end:].lstrip().startswith("else")
    rejected, _ = block_after(text[end:], "else")
    assert "map_incremental" not in rejected
    assert "add_point_size = 0;" in rejected
    assert "kdtree_incremental_time = 0.0;" in rejected
    assert "return;" not in rejected
