from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable, Sequence


@dataclass(slots=True)
class Pose2D:
    """2차원 로봇 자세. x/y는 m, theta는 rad."""

    x: float = 0.0
    y: float = 0.0
    theta: float = 0.0


def clamp(value: float, low: float, high: float) -> float:
    if low > high:
        raise ValueError("low must not exceed high")
    return max(low, min(high, float(value)))


def wrap_angle(angle_rad: float) -> float:
    """각도를 [-pi, pi)로 정규화합니다."""

    return (float(angle_rad) + math.pi) % (2.0 * math.pi) - math.pi


def wheel_percent_to_mps(
    percent: float,
    wheel_radius_m: float = 0.033,
    max_rpm: float = 93.75,
) -> float:
    """교육용 근사: Beagle 바퀴 명령 비율을 선속도(m/s)로 바꿉니다.

    실제 장비에서는 반드시 거리-엔코더 보정값으로 교체하세요.
    """

    pct = clamp(percent, -100.0, 100.0)
    return (pct / 100.0) * 2.0 * math.pi * wheel_radius_m * (max_rpm / 60.0)


def mps_to_wheel_percent(
    speed_mps: float,
    wheel_radius_m: float = 0.033,
    max_rpm: float = 93.75,
) -> float:
    max_mps = 2.0 * math.pi * wheel_radius_m * (max_rpm / 60.0)
    if max_mps <= 0.0:
        raise ValueError("wheel_radius_m and max_rpm must be positive")
    return clamp(100.0 * float(speed_mps) / max_mps, -100.0, 100.0)


def wheel_speeds_to_twist(
    left_mps: float,
    right_mps: float,
    wheel_base_m: float = 0.0956,
) -> tuple[float, float]:
    """좌우 바퀴 선속도 -> (선속도, 각속도)."""

    if wheel_base_m <= 0.0:
        raise ValueError("wheel_base_m must be positive")
    linear = (float(left_mps) + float(right_mps)) / 2.0
    angular = (float(right_mps) - float(left_mps)) / wheel_base_m
    return linear, angular


def twist_to_wheel_speeds(
    linear_mps: float,
    angular_rps: float,
    wheel_base_m: float = 0.0956,
) -> tuple[float, float]:
    """(선속도, 각속도) -> 좌우 바퀴 선속도."""

    half = wheel_base_m / 2.0
    return linear_mps - angular_rps * half, linear_mps + angular_rps * half


def twist_to_wheel_percent(
    linear_mps: float,
    angular_rps: float,
    wheel_base_m: float = 0.0956,
    wheel_radius_m: float = 0.033,
    max_rpm: float = 93.75,
) -> tuple[float, float]:
    left_mps, right_mps = twist_to_wheel_speeds(linear_mps, angular_rps, wheel_base_m)
    return (
        mps_to_wheel_percent(left_mps, wheel_radius_m, max_rpm),
        mps_to_wheel_percent(right_mps, wheel_radius_m, max_rpm),
    )


def integrate_wheel_distances(
    pose: Pose2D,
    left_distance_m: float,
    right_distance_m: float,
    wheel_base_m: float = 0.0956,
    gyro_delta_rad: float | None = None,
    gyro_weight: float = 0.65,
) -> Pose2D:
    """한 샘플 구간의 좌우 이동거리로 자세를 갱신합니다.

    gyro_delta_rad가 주어지면 엔코더 회전량과 상보적으로 결합합니다.
    """

    if wheel_base_m <= 0.0:
        raise ValueError("wheel_base_m must be positive")
    d_left = float(left_distance_m)
    d_right = float(right_distance_m)
    distance_m = (d_left + d_right) / 2.0
    wheel_delta = (d_right - d_left) / wheel_base_m
    delta_theta = wheel_delta
    if gyro_delta_rad is not None and math.isfinite(gyro_delta_rad):
        weight = clamp(gyro_weight, 0.0, 1.0)
        delta_theta = weight * float(gyro_delta_rad) + (1.0 - weight) * wheel_delta
    mid_theta = pose.theta + delta_theta / 2.0
    return Pose2D(
        pose.x + distance_m * math.cos(mid_theta),
        pose.y + distance_m * math.sin(mid_theta),
        wrap_angle(pose.theta + delta_theta),
    )


def integrate_velocity(
    pose: Pose2D,
    left_mps: float,
    right_mps: float,
    dt_s: float,
    wheel_base_m: float = 0.0956,
    gyro_z_dps: float | None = None,
    gyro_bias_dps: float = 0.0,
    gyro_weight: float = 0.65,
) -> Pose2D:
    if dt_s <= 0.0:
        return Pose2D(pose.x, pose.y, pose.theta)
    gyro_delta = None
    if gyro_z_dps is not None and math.isfinite(float(gyro_z_dps)):
        gyro_delta = math.radians((float(gyro_z_dps) - gyro_bias_dps) * dt_s)
    return integrate_wheel_distances(
        pose,
        float(left_mps) * dt_s,
        float(right_mps) * dt_s,
        wheel_base_m,
        gyro_delta,
        gyro_weight,
    )


def polar_to_xy(
    distances_mm: Sequence[float],
    *,
    angle_offset_deg: float = 0.0,
    min_mm: float = 50.0,
    max_mm: float = 5000.0,
) -> list[tuple[float, float]]:
    """0°=전방(+x), 90°=좌측(+y)인 로봇 좌표(mm)로 변환합니다."""

    count = len(distances_mm)
    if count == 0:
        return []
    step_deg = 360.0 / count
    points: list[tuple[float, float]] = []
    for index, raw in enumerate(distances_mm):
        try:
            distance_mm = float(raw)
        except (TypeError, ValueError):
            continue
        if not math.isfinite(distance_mm) or not (min_mm <= distance_mm <= max_mm):
            continue
        theta = math.radians(index * step_deg + angle_offset_deg)
        points.append((distance_mm * math.cos(theta), distance_mm * math.sin(theta)))
    return points


def transform_point_mm(point_mm: tuple[float, float], pose: Pose2D) -> tuple[float, float]:
    """로봇 좌표(mm)의 한 점을 world 좌표(m)로 변환합니다."""

    x_local_m, y_local_m = point_mm[0] / 1000.0, point_mm[1] / 1000.0
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return (
        pose.x + c * x_local_m - s * y_local_m,
        pose.y + s * x_local_m + c * y_local_m,
    )


def transform_points_mm(
    points_mm: Iterable[tuple[float, float]],
    pose: Pose2D,
) -> list[tuple[float, float]]:
    return [transform_point_mm(point, pose) for point in points_mm]


def world_to_robot(point_world_m: tuple[float, float], pose: Pose2D) -> tuple[float, float]:
    """world 점을 로봇 좌표(m)로 변환합니다."""

    dx, dy = point_world_m[0] - pose.x, point_world_m[1] - pose.y
    c, s = math.cos(pose.theta), math.sin(pose.theta)
    return c * dx + s * dy, -s * dx + c * dy


def euclidean(a: tuple[float, float], b: tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def integrate_differential_drive(
    pose: Pose2D,
    left_mps: float,
    right_mps: float,
    dt: float,
    wheel_base_m: float = 0.0956,
    gyro_z_dps: float | None = None,
    gyro_weight: float = 0.65,
) -> Pose2D:
    """이전 저장소와의 호환 별칭. integrate_velocity와 같은 중점법 적분입니다."""
    return integrate_velocity(
        pose, left_mps, right_mps, dt,
        wheel_base_m=wheel_base_m,
        gyro_z_dps=gyro_z_dps,
        gyro_weight=gyro_weight,
    )
