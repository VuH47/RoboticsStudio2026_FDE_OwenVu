#!/usr/bin/env python3
"""Save active SLAM maps for Husky and/or Parrot to disk (.yaml + .pgm + .posegraph).

Can be executed directly via Python or as a ROS 2 package executable:
    python3 scripts/save_slam_map.py --robot husky1
    python3 scripts/save_slam_map.py --robot parrot1
    python3 scripts/save_slam_map.py --robot both
    ros2 run 41068_ignition_bringup save_slam_map.py --robot both

Saves the maps into the package 'maps/' directory, and copies them to the
Windows repository workspace if running under WSL.
"""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import shutil
import subprocess
import sys
import time


def find_maps_dirs() -> tuple[Path, Path | None]:
    """Find local maps directory and optional Windows host maps directory."""
    script_dir = Path(__file__).resolve().parent
    pkg_root = script_dir.parent

    # Local package maps directory
    local_maps = pkg_root / 'maps'
    local_maps.mkdir(parents=True, exist_ok=True)

    # If running inside WSL, locate the mounted Windows directory
    win_maps = None
    win_mount = Path('/mnt/c/Users/Asus/Downloads/41068RoboticsStudio1pring2026/41068_ignition_bringup_v1/41068_ignition_bringup/maps')
    if win_mount.parent.exists():
        win_mount.mkdir(parents=True, exist_ok=True)
        win_maps = win_mount

    return local_maps, win_maps


def get_active_map_topics() -> list[str]:
    """Query ROS 2 for active map topics."""
    try:
        res = subprocess.run(
            ['ros2', 'topic', 'list'],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )
        if res.returncode == 0:
            return [line.strip() for line in res.stdout.splitlines() if line.strip()]
    except Exception:
        pass
    return []


def save_single_robot_map(
    robot: str,
    output_dir: Path,
    win_maps_dir: Path | None = None,
    name_override: str | None = None,
    timeout: float = 15.0,
    serialize_posegraph: bool = True,
) -> bool:
    """Save map for a single robot namespace."""
    map_topic = f'/{robot}/map'
    base_name = name_override if name_override else f'{robot}_map'
    output_base = output_dir / base_name

    print(f"\n==========================================")
    print(f" Saving SLAM Map for robot: '{robot}'")
    print(f" Topic:      {map_topic}")
    print(f" Target:     {output_base}.yaml / .pgm")
    print(f"==========================================")

    # 1. Run nav2_map_server map_saver_cli
    cmd = [
        'ros2', 'run', 'nav2_map_server', 'map_saver_cli',
        '-t', map_topic,
        '-f', str(output_base),
        '--mode', 'trinary',
    ]

    print(f"Running: {' '.join(cmd)}")
    try:
        proc = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
        if proc.returncode != 0:
            print(f"ERROR: map_saver_cli failed with exit code {proc.returncode}")
            if proc.stderr:
                print(f"Stderr:\n{proc.stderr.strip()}")
            if proc.stdout:
                print(f"Stdout:\n{proc.stdout.strip()}")
            return False
    except subprocess.TimeoutExpired:
        print(f"ERROR: Timed out after {timeout}s waiting for map on {map_topic}.")
        print("Make sure SLAM is running (slam:=true) and the robot has produced at least one scan.")
        return False
    except Exception as e:
        print(f"ERROR: Failed to run map_saver_cli: {e}")
        return False

    yaml_file = output_dir / f'{base_name}.yaml'
    pgm_file = output_dir / f'{base_name}.pgm'

    if not yaml_file.exists() or not pgm_file.exists():
        print(f"ERROR: Expected map files were not created: {yaml_file}")
        return False

    yaml_size = yaml_file.stat().st_size
    pgm_size = pgm_file.stat().st_size
    print(f"SUCCESS: OccupancyGrid saved!")
    print(f"  YAML: {yaml_file} ({yaml_size} bytes)")
    print(f"  PGM:  {pgm_file} ({pgm_size} bytes)")

    # 2. Try serializing SLAM Toolbox pose graph if service exists
    if serialize_posegraph:
        serialize_srv = f'/{robot}/slam_toolbox/serialize_map'
        print(f"Attempting to serialize SLAM pose graph via {serialize_srv}...")
        try:
            srv_call = [
                'ros2', 'service', 'call',
                serialize_srv,
                'slam_toolbox/srv/SerializePoseGraph',
                f'{{filename: {{data: "{str(output_base)}"}} }}',
            ]
            s_proc = subprocess.run(srv_call, capture_output=True, text=True, timeout=5.0)
            if s_proc.returncode == 0 and 'result=0' in s_proc.stdout:
                print(f"SUCCESS: Pose graph serialized to {output_base}.posegraph")
            else:
                print(f"Note: SLAM Toolbox serialize_map service returned: {s_proc.stdout.strip()}")
        except Exception:
            print(f"Note: Could not call {serialize_srv} (optional).")

    # 3. Copy to Windows repository if available
    if win_maps_dir and win_maps_dir != output_dir:
        try:
            for ext in ['.yaml', '.pgm', '.posegraph', '.data']:
                src = output_dir / f'{base_name}{ext}'
                if src.exists():
                    dst = win_maps_dir / f'{base_name}{ext}'
                    shutil.copy2(src, dst)
                    print(f"Synced to Windows: {dst}")
        except Exception as e:
            print(f"Warning: Failed to copy to Windows directory: {e}")

    return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Save active SLAM maps for Husky and/or Parrot.")
    parser.add_argument(
        '-r', '--robot',
        choices=['husky1', 'parrot1', 'both', 'auto'],
        default='auto',
        help="Which robot map to save: 'husky1', 'parrot1', 'both', or 'auto' (detect active map topics). Default: auto",
    )
    parser.add_argument(
        '-n', '--name',
        default=None,
        help="Custom base filename for the map (e.g. 'snow_forest_map'). Default: <robot>_map",
    )
    parser.add_argument(
        '-o', '--output-dir',
        default=None,
        help="Directory to save the maps. Defaults to 'maps/' in the package.",
    )
    parser.add_argument(
        '-t', '--timeout',
        type=float,
        default=15.0,
        help="Timeout in seconds to wait for map data. Default: 15.0s",
    )
    parser.add_argument(
        '--no-serialize',
        action='store_true',
        help="Skip saving SLAM Toolbox pose graph (.posegraph).",
    )

    args = parser.parse_args()

    local_maps, win_maps = find_maps_dirs()
    output_dir = Path(args.output_dir).resolve() if args.output_dir else local_maps
    output_dir.mkdir(parents=True, exist_ok=True)

    robots_to_save: list[str] = []
    if args.robot == 'both':
        robots_to_save = ['husky1', 'parrot1']
    elif args.robot in ('husky1', 'parrot1'):
        robots_to_save = [args.robot]
    else:  # auto
        active_topics = get_active_map_topics()
        if '/husky1/map' in active_topics:
            robots_to_save.append('husky1')
        if '/parrot1/map' in active_topics:
            robots_to_save.append('parrot1')

        if not robots_to_save:
            print("No active /husky1/map or /parrot1/map topics detected.")
            print("Defaulting to saving both ['husky1', 'parrot1']...")
            robots_to_save = ['husky1', 'parrot1']

    success_all = True
    for robot in robots_to_save:
        ok = save_single_robot_map(
            robot=robot,
            output_dir=output_dir,
            win_maps_dir=win_maps,
            name_override=args.name,
            timeout=args.timeout,
            serialize_posegraph=not args.no_serialize,
        )
        if not ok:
            success_all = False

    print("\n------------------------------------------")
    if success_all:
        print(" All requested SLAM maps saved successfully!")
    else:
        print(" One or more map saves failed or timed out.")
    print("------------------------------------------\n")

    return 0 if success_all else 1


if __name__ == '__main__':
    sys.exit(main())
