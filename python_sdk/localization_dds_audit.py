"""Read-only Linux /proc audit used before ground-only DDS recovery.

Runs as root solely to inspect other users' process maps. Imports no ROS or FC
modules, signals no process, and never writes or removes shared memory files.
"""

import argparse
import json
from pathlib import Path


def inspect_dds_owners(released_task_pid, proc_root=Path("/proc")):
    owners, unreadable = [], []
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            maps = (proc / "maps").read_text()
            shm = "/dev/shm/fastrtps" in maps or "/dev/shm/fastdds" in maps
            dds = any(lib in maps for lib in ("libfastrtps.so", "libfastdds.so", "libddsc.so"))
            if shm or (dds and int(proc.name) != released_task_pid):
                owners.append({"pid": int(proc.name), "name": (proc / "comm").read_text().strip()})
        except (FileNotFoundError, ProcessLookupError):
            continue  # Process exited while the snapshot was being read.
        except OSError:
            unreadable.append(int(proc.name))
    return {"owners": owners, "unreadable": unreadable}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--released-task-pid", type=int, required=True)
    args = parser.parse_args()
    print(json.dumps(inspect_dds_owners(args.released_task_pid)))
