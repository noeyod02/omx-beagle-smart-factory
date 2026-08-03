from __future__ import annotations

import math
import time

from .robot import SafeBeagle


def valid_lidar_distance(value: float, min_mm: float = 40.0, max_mm: float = 5000.0) -> bool:
    try:
        value = float(value)
    except (TypeError, ValueError, OverflowError):
        return False
    return math.isfinite(value) and min_mm <= value <= max_mm


def calibrate_gyro_bias(robot: SafeBeagle, seconds: float = 0.8, sample_s: float = 0.02) -> float:
    values: list[float] = []
    robot.stop()
    deadline = time.monotonic() + max(0.1, seconds)
    while time.monotonic() < deadline:
        values.append(float(robot.gyroscope_z()))
        time.sleep(sample_s)
    return sum(values) / max(1, len(values))


def turn_degrees(
    robot: SafeBeagle,
    target_deg: float,
    *,
    speed: float = 11.0,
    timeout_s: float = 5.0,
    tolerance_deg: float = 2.0,
) -> float:
    """자이로 적분으로 목표 각도만큼 회전합니다. 양수는 반시계 방향입니다."""

    bias = calibrate_gyro_bias(robot)
    integrated = 0.0
    previous = time.monotonic()
    direction = 1.0 if target_deg >= 0.0 else -1.0
    deadline = time.monotonic() + timeout_s
    robot.wheels(-speed * direction, speed * direction)
    try:
        while time.monotonic() < deadline:
            now = time.monotonic()
            dt = min(0.1, max(0.0, now - previous))
            previous = now
            integrated += (float(robot.gyroscope_z()) - bias) * dt
            if abs(target_deg - integrated) <= tolerance_deg or abs(integrated) >= abs(target_deg):
                break
            time.sleep(0.02)
    finally:
        robot.stop()
    return integrated


def center_one_axis(
    robot: SafeBeagle,
    *,
    tolerance_mm: float = 20.0,
    slow_zone_mm: float = 55.0,
    fast_speed: float = 15.0,
    slow_speed: float = 8.0,
    stable_samples: int = 4,
    timeout_s: float = 7.0,
    verbose: bool = True,
) -> bool:
    """front-rear 차이를 줄여 현재 heading 방향의 중심을 찾습니다."""

    stable = 0
    deadline = time.monotonic() + timeout_s
    try:
        while time.monotonic() < deadline:
            front = float(robot.front_lidar())
            rear = float(robot.rear_lidar())
            if not (valid_lidar_distance(front) and valid_lidar_distance(rear)):
                robot.stop()
                return False
            error = front - rear
            if abs(error) <= tolerance_mm:
                stable += 1
                robot.stop()
                if stable >= stable_samples:
                    return True
            else:
                stable = 0
                speed = slow_speed if abs(error) < slow_zone_mm else fast_speed
                if error > 0:
                    robot.wheels(speed, speed)
                else:
                    robot.wheels(-speed, -speed)
            if verbose:
                print(f"front={front:6.1f} rear={rear:6.1f} error={error:+6.1f} stable={stable}")
            time.sleep(0.05)
    finally:
        robot.stop()
    return False
