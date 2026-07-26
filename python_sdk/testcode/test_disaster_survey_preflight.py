from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bus.preflight import navigation_tf_is_fresh


class _FreshnessSource:
    def __init__(self, fresh):
        self.fresh = fresh
        self.max_ages = []

    def is_transform_fresh(self, max_age):
        self.max_ages.append(max_age)
        return self.fresh


class _Navigation:
    def __init__(self, tf_fresh, pose_fresh):
        self.mapper = _FreshnessSource(tf_fresh)
        self._pose_fresh = pose_fresh
        self.max_ages = []

    def pose_is_fresh(self, max_age):
        self.max_ages.append(max_age)
        return self._pose_fresh


class DisasterSurveyPreflightTests(unittest.TestCase):
    def test_requires_both_tf_and_navigation_pose_to_be_fresh(self):
        self.assertTrue(navigation_tf_is_fresh(_Navigation(True, True), 0.5))
        self.assertFalse(navigation_tf_is_fresh(_Navigation(False, True), 0.5))
        self.assertFalse(navigation_tf_is_fresh(_Navigation(True, False), 0.5))

    def test_forwards_the_same_maximum_age_to_both_checks(self):
        navigation = _Navigation(True, True)
        self.assertTrue(navigation_tf_is_fresh(navigation, 0.25))
        self.assertEqual([0.25], navigation.mapper.max_ages)
        self.assertEqual([0.25], navigation.max_ages)

    def test_rejects_non_positive_maximum_age(self):
        with self.assertRaises(ValueError):
            navigation_tf_is_fresh(_Navigation(True, True), 0)


if __name__ == "__main__":
    unittest.main()
