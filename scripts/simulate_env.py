#!/usr/bin/env python3
"""
Gazebo Environment Spawner

Spawns simple objects into a Gazebo simulation world to create a test
maze-like environment for the SLAM robot.

Note: This is a placeholder script to illustrate structure. Students can
extend this to use Gazebo services to spawn models (gazebo_ros). If Gazebo
is not used, this can generate RViz markers or a static map for testing.
"""
import math
import time


def spawn_box(name: str, x: float, y: float, yaw: float = 0.0):
    print(f"[simulate_env] Spawning box '{name}' at ({x:.2f}, {y:.2f}), yaw={yaw:.2f} rad")


def main():
    print("[simulate_env] Starting environment setup...")
    # Example layout: a corridor and a room
    spawn_box('wall_1', 1.0, 0.0)
    spawn_box('wall_2', 2.0, 0.0)
    spawn_box('pillar', 1.5, 1.2)
    print("[simulate_env] Environment setup complete.")


if __name__ == '__main__':
    main()
