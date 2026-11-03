# Copyright 2026 Azer Ben Ali
# SPDX-License-Identifier: Apache-2.0
"""Scenario loading and the path-following control law, independent of ROS."""

from dataclasses import dataclass, field
import math
import os

import numpy as np
import yaml

ROBOT_ARGS = ('lidar_channels', 'lidar_horizontal_samples', 'lidar_rate_hz', 'camera_rate_hz',
              'odom_wheel_radius_scale', 'odom_wheel_separation_scale')


@dataclass
class Scenario:
    name: str
    description: str
    spawn: dict
    robot: dict = field(default_factory=dict)
    waypoints: list = field(default_factory=list)
    speed_mps: float = 0.5
    max_duration_s: float = 600.0

    @staticmethod
    def load(path):
        with open(path) as f:
            data = yaml.safe_load(f)
        name = os.path.splitext(os.path.basename(path))[0]
        if data.get('name', name) != name:
            raise ValueError(f'{path}: name "{data["name"]}" does not match the file name')
        unknown = set(data.get('robot', {})) - set(ROBOT_ARGS)
        if unknown:
            raise ValueError(f'{path}: unknown robot arguments {sorted(unknown)}')
        route = data.get('route') or {}
        scenario = Scenario(
            name=name,
            description=data.get('description', ''),
            spawn={k: float(data['spawn'][k]) for k in ('x', 'y', 'yaw')},
            robot=data.get('robot', {}),
            waypoints=[tuple(map(float, p)) for p in route.get('waypoints', [])],
            speed_mps=float(route.get('speed_mps', 0.5)),
            max_duration_s=float(route.get('max_duration_s', 600.0)))
        if scenario.waypoints and len(scenario.waypoints) < 2:
            raise ValueError(f'{path}: a route needs at least two waypoints')
        if not 0.0 < scenario.speed_mps <= 1.5:
            raise ValueError(f'{path}: speed_mps must be in (0, 1.5] (MecanumDrive limit)')
        return scenario


def densify(waypoints, spacing=0.1):
    """Polyline through the waypoints resampled every `spacing` metres."""
    points = [np.asarray(waypoints[0], dtype=float)]
    for a, b in zip(waypoints[:-1], waypoints[1:]):
        a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
        n = max(1, int(math.ceil(np.linalg.norm(b - a) / spacing)))
        points.extend(a + (b - a) * k / n for k in range(1, n + 1))
    return np.array(points)


@dataclass
class PursuitState:
    progress: int = 0
    done: bool = False


@dataclass
class PursuitConfig:
    speed_mps: float = 0.5
    lookahead_m: float = 1.0
    max_angular_rps: float = 1.0
    # Above this heading error the robot turns in place instead of arcing,
    # which keeps it inside 3 m aisles at 90-degree corners.
    rotate_in_place_rad: float = 0.7
    goal_tolerance_m: float = 0.3
    # Progress may only advance this far per update, so a loop whose end
    # passes its start can never skip ahead to the finish.
    search_window_m: float = 3.0
    spacing_m: float = 0.1


def pursuit_command(x, y, yaw, path, state: PursuitState, cfg: PursuitConfig):
    """
    Pure-pursuit step: returns (linear [m/s], angular [rad/s]) and updates state.

    Curvature of the arc through the lookahead point is k = 2 sin(alpha) / L,
    with alpha the bearing of that point relative to the heading.
    """
    if state.done:
        return 0.0, 0.0
    pos = np.array([x, y])
    window_end = min(len(path), state.progress + int(cfg.search_window_m / cfg.spacing_m) + 1)
    window = path[state.progress:window_end]
    state.progress += int(np.argmin(np.linalg.norm(window - pos, axis=1)))

    to_goal = np.linalg.norm(path[-1] - pos)
    if state.progress >= len(path) - 1 - int(cfg.lookahead_m / cfg.spacing_m) and \
            to_goal < cfg.goal_tolerance_m:
        state.done = True
        return 0.0, 0.0

    target = path[min(len(path) - 1, state.progress + int(cfg.lookahead_m / cfg.spacing_m))]
    alpha = math.atan2(target[1] - y, target[0] - x) - yaw
    alpha = math.atan2(math.sin(alpha), math.cos(alpha))
    if abs(alpha) > cfg.rotate_in_place_rad:
        return 0.0, math.copysign(cfg.max_angular_rps, alpha)
    # Slow down approaching the goal so the final pose is reached without overshoot.
    v = min(cfg.speed_mps * math.cos(alpha), max(0.1, 0.8 * to_goal))
    omega = 2.0 * v * math.sin(alpha) / cfg.lookahead_m
    return v, max(-cfg.max_angular_rps, min(cfg.max_angular_rps, omega))
