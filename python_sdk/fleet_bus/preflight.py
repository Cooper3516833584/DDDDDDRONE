"""Hardware-free preflight readiness checks shared by task entry points."""


def navigation_tf_is_fresh(navigation, max_age_seconds=0.5):
    """Return whether both the map TF and derived navigation pose are fresh."""
    if max_age_seconds <= 0:
        raise ValueError("max_age_seconds must be positive")
    mapper = getattr(navigation, "mapper", None)
    mapper_check = getattr(mapper, "is_transform_fresh", None)
    pose_check = getattr(navigation, "pose_is_fresh", None)
    return bool(
        callable(mapper_check)
        and callable(pose_check)
        and mapper_check(max_age_seconds)
        and pose_check(max_age_seconds)
    )
