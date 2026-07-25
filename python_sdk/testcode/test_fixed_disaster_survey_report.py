from pathlib import Path
import sys
import unittest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from fleet_bus.models import SurveyFlags, TerrainCode
from testcode.fixed_disaster_survey_report import (
    build_survey_state,
    coordinate_to_cell,
)


class FixedDisasterSurveyTests(unittest.TestCase):
    def test_requested_coordinates_map_to_row_major_cells(self):
        self.assertEqual((0, 1), coordinate_to_cell(70, 0))
        self.assertEqual((1, 2), coordinate_to_cell(140, 70))

    def test_default_report_contains_water_and_wildfire(self):
        state = build_survey_state()
        self.assertEqual(int(SurveyFlags.COMPLETE), state.survey_flags)
        self.assertEqual((1, 1, 2), (
            state.wildfire_event_id,
            state.wildfire_row,
            state.wildfire_col,
        ))
        self.assertEqual(int(TerrainCode.RIVER), state.terrain_codes[1])
        self.assertEqual(int(TerrainCode.WILDFIRE), state.terrain_codes[7])
        self.assertEqual(15, len(state.terrain_codes))

    def test_rejects_non_grid_and_overlapping_coordinates(self):
        with self.assertRaises(ValueError):
            coordinate_to_cell(71, 0)
        with self.assertRaises(ValueError):
            coordinate_to_cell(350, 0)
        with self.assertRaises(ValueError):
            build_survey_state(70, 0, 70, 0)


if __name__ == "__main__":
    unittest.main()
