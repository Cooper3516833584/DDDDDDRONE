"""Flight-computer GUI: display existing ROS localization without owning services.

Run on the NoMachine desktop or directly from VSCode. Task restart helpers are
available from fastlio_control and use the existing task FC/Navigation objects.
"""

import sys

from fastlio_control import (
    restart_fastlio_for_task, restart_and_calibrate_fastlio, restart_localization_for_task,
)
from lio_trajectory_viewer import main


if __name__ == "__main__":
    if sys.platform != "linux":
        raise RuntimeError("Use lio_trajectory_viewer.py for the Windows local radar")
    if "--new-map" in sys.argv:
        raise RuntimeError("Aircraft localization is systemd-managed; use the ground task restart API")
    main()
