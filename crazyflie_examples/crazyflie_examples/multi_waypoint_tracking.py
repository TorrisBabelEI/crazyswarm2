#!/usr/bin/env python
"""
Multi Crazyflie Waypoint Tracking

Follows per-drone CSV trajectories using linear interpolation and goTo commands.
CSV format (rows x columns, no header):
  Row 0: time in seconds
  Row 1: x positions
  Row 2: y positions
  Row 3: z positions
  Rows 4-6: vx, vy, vz (ignored)

Each drone reads its own CSV: traj/ref/cf_01_traj_ref.csv, cf_02_traj_ref.csv, ...
Drone indices are set via CF_INDICES below.
"""

from pathlib import Path

import numpy as np
from crazyflie_py import Crazyswarm

# --- Configuration ---
CF_INDICES = [1, 2]          # which cf_XX_traj_ref.csv files to load
FLIGHT_TIME = 60.0           # total trajectory duration (seconds)
TAKEOFF_HEIGHT = 1.0         # metres
TAKEOFF_DURATION = 2.5       # seconds
GOTO_DURATION = 0.1          # seconds per goTo command (controls responsiveness)
TRAJ_DIR = Path('traj/ref')  # relative to working directory when launched


def load_trajectory(csv_path):
    """Return (t, positions) where positions is (3, N)."""
    data = np.loadtxt(csv_path, delimiter=',')
    t = data[0, :]
    pos = data[1:4, :]
    # Normalise time to [0, FLIGHT_TIME]
    t = (t - t[0]) / (t[-1] - t[0]) * FLIGHT_TIME
    return t, pos


def interp_pos(t_query, t_ref, pos_ref):
    """Linear interpolation clamped to trajectory endpoints."""
    t_query = float(np.clip(t_query, t_ref[0], t_ref[-1]))
    return np.array([
        float(np.interp(t_query, t_ref, pos_ref[i])) for i in range(3)
    ])


def main():
    swarm = Crazyswarm()
    timeHelper = swarm.timeHelper
    cfs = swarm.allcfs.crazyflies

    # Load one trajectory per drone (matched by list order)
    trajs = []
    for idx in CF_INDICES:
        csv_path = TRAJ_DIR / f'cf_{idx:02d}_traj_ref.csv'
        t_ref, pos_ref = load_trajectory(csv_path)
        trajs.append((t_ref, pos_ref))

    # Takeoff
    for cf in cfs:
        cf.takeoff(targetHeight=TAKEOFF_HEIGHT, duration=TAKEOFF_DURATION)
    timeHelper.sleep(TAKEOFF_DURATION + 1.0)

    # Move each drone to its trajectory start position
    for cf, (t_ref, pos_ref) in zip(cfs, trajs):
        start = interp_pos(0.0, t_ref, pos_ref)
        cf.goTo(start, 0.0, 2.0)
    timeHelper.sleep(2.5)

    # Trajectory tracking loop
    t_elapsed = 0.0
    while t_elapsed <= FLIGHT_TIME:
        for cf, (t_ref, pos_ref) in zip(cfs, trajs):
            target = interp_pos(t_elapsed, t_ref, pos_ref)
            cf.goTo(target, 0.0, GOTO_DURATION)
        timeHelper.sleep(GOTO_DURATION)
        t_elapsed += GOTO_DURATION

    # Land
    for cf in cfs:
        cf.land(targetHeight=0.06, duration=2.0)
    timeHelper.sleep(3.0)


if __name__ == '__main__':
    main()
