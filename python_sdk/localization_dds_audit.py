"""Read-only Linux /proc audit used before ground-only DDS recovery.

Runs as root solely to inspect other users' process maps. Imports no ROS or FC
modules, signals no process, and never writes or removes shared memory files.
"""

import argparse
import json
from pathlib import Path


def inspect_dds_owners(released_task_pid, proc_root=Path("/proc"), *, details=False):
    owners, unreadable = [], []
    for proc in proc_root.iterdir():
        if not proc.name.isdigit():
            continue
        try:
            maps = (proc / "maps").read_text()
            shm = "/dev/shm/fastrtps" in maps or "/dev/shm/fastdds" in maps
            dds = any(lib in maps for lib in ("libfastrtps.so", "libfastdds.so", "libddsc.so"))
            if shm or (dds and int(proc.name) != released_task_pid):
                owner = {"pid": int(proc.name), "name": (proc / "comm").read_text().strip()}
                if details:
                    owner["cgroups"] = [line.split(":", 2)[-1] for line in
                                        (proc / "cgroup").read_text().splitlines()]
                    owner["deleted_shm"] = [line.split()[-2] for line in maps.splitlines()
                                            if "/dev/shm/" in line and
                                            ("fastrtps" in line or "fastdds" in line) and
                                            line.endswith(" (deleted)")]
                owners.append(owner)
        except (FileNotFoundError, ProcessLookupError):
            continue  # Process exited while the snapshot was being read.
        except OSError:
            unreadable.append(int(proc.name))
    return {"owners": owners, "unreadable": unreadable}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--released-task-pid", type=int, required=True)
    parser.add_argument("--details", action="store_true")
    args = parser.parse_args()
    print(json.dumps(inspect_dds_owners(args.released_task_pid, details=args.details)))
