from __future__ import annotations

import atexit
import math
import random
import signal
import time
from typing import Any

from .geometry import Pose2D, integrate_velocity, wheel_percent_to_mps
from .lidar import cardinal_distances, sanitize_scan

Segment = tuple[float, float, float, float]


def rectangle_segments(x0: float, y0: float, x1: float, y1: float) -> list[Segment]:
    return [(x0, y0, x1, y0), (x1, y0, x1, y1), (x1, y1, x0, y1), (x0, y1, x0, y0)]


def build_scene(name: str) -> tuple[list[Segment], Pose2D]:
    name = name.lower().strip()
    if name == "room":
        return rectangle_segments(0.0, 0.0, 1.2, 0.9), Pose2D(0.32, 0.26, 0.0)
    if name in {"room_exit", "escape"}:
        walls: list[Segment] = [
            (0.0, 0.0, 1.2, 0.0),
            (1.2, 0.0, 1.2, 0.28),
            (1.2, 0.62, 1.2, 0.90),
            (1.2, 0.90, 0.0, 0.90),
            (0.0, 0.90, 0.0, 0.0),
        ]
        return walls, Pose2D(0.42, 0.43, 0.0)
    if name == "corridor":
        return [
            (0.0, 0.28, 2.3, 0.28),
            (0.0, 0.68, 2.3, 0.68),
            (0.0, 0.28, 0.0, 0.68),
            (2.3, 0.28, 2.3, 0.68),
        ], Pose2D(0.25, 0.46, 0.0)
    if name == "maze":
        walls = [
            (0.0, 0.2, 2.0, 0.2),
            (0.0, 0.7, 1.55, 0.7),
            (1.55, 0.7, 1.55, 1.7),
            (2.0, 0.2, 2.0, 1.2),
            (1.55, 1.7, 3.2, 1.7),
            (2.0, 1.2, 3.2, 1.2),
            (0.0, 0.2, 0.0, 0.7),
            (3.2, 1.2, 3.2, 1.7),
        ]
        return walls, Pose2D(0.25, 0.47, 0.0)
    if name in {"obstacles", "default"}:
        walls = rectangle_segments(0.0, 0.0, 4.0, 3.0)
        walls += rectangle_segments(1.8, 0.5, 2.15, 2.1)
        walls += rectangle_segments(2.8, 1.9, 3.35, 2.25)
        return walls, Pose2D(0.75, 0.75, 0.0)
    if name == "narrow":
        # 실물 실습 맵과 같은 치수의 ㄷ자 미로:
        # 한 변 40cm, 통로 폭 13cm (로봇 폭 10cm + 양옆 여유 1.5cm).
        walls = rectangle_segments(0.0, 0.0, 0.40, 0.40)
        walls += rectangle_segments(0.13, 0.13, 0.27, 0.40)
        return walls, Pose2D(0.065, 0.32, -math.pi / 2.0)
    if name == "open":
        return [], Pose2D(0.0, 0.0, 0.0)
    raise ValueError(f"unknown mock scene: {name}")


def ray_segment_distance(x: float, y: float, dx: float, dy: float, segment: Segment) -> float:
    x1, y1, x2, y2 = segment
    sx, sy = x2 - x1, y2 - y1
    denominator = dx * sy - dy * sx
    if abs(denominator) < 1e-12:
        return math.inf
    qx, qy = x1 - x, y1 - y
    t = (qx * sy - qy * sx) / denominator
    u = (qx * dy - qy * dx) / denominator
    if t >= 0.0 and 0.0 <= u <= 1.0:
        return t
    return math.inf


class MockBeagle:
    """코드 흐름과 알고리즘 분기를 점검하는 간이 2D Beagle."""

    LEFT_WHEEL = 0
    RIGHT_WHEEL = 1

    def __init__(self, scene: str = "default", *, seed: int = 7) -> None:
        self.scene = scene
        self.segments, self.pose = build_scene(scene)
        self.random = random.Random(seed)
        self.left_percent = 0.0
        self.right_percent = 0.0
        self.last_update = time.monotonic()
        self.left_distance_m = 0.0
        self.right_distance_m = 0.0
        self.lidar_started = False
        self.gyro_bias_dps = 0.35

    def _update(self) -> None:
        now = time.monotonic()
        dt = min(0.15, max(0.0, now - self.last_update))
        self.last_update = now
        left_mps = wheel_percent_to_mps(self.left_percent)
        right_mps = wheel_percent_to_mps(self.right_percent)
        self.pose = integrate_velocity(self.pose, left_mps, right_mps, dt)
        self.left_distance_m += left_mps * dt
        self.right_distance_m += right_mps * dt

    def wheels(self, left: float, right: float) -> None:
        self._update()
        self.left_percent = float(left)
        self.right_percent = float(right)

    def write(self, channel: int, value: float) -> None:
        if channel == self.LEFT_WHEEL:
            self.wheels(value, self.right_percent)
        elif channel == self.RIGHT_WHEEL:
            self.wheels(self.left_percent, value)

    def stop(self) -> None:
        self.wheels(0.0, 0.0)

    def start_lidar(self) -> None:
        self.lidar_started = True

    def wait_until_lidar_ready(self) -> None:
        self.lidar_started = True
        time.sleep(0.02)

    def is_lidar_ready(self) -> bool:
        return self.lidar_started

    def lidar(self) -> list[int]:
        self._update()
        result: list[int] = []
        for degree in range(360):
            theta = self.pose.theta + math.radians(degree)
            dx, dy = math.cos(theta), math.sin(theta)
            distance = min(
                (ray_segment_distance(self.pose.x, self.pose.y, dx, dy, segment) for segment in self.segments),
                default=5.0,
            )
            if not math.isfinite(distance):
                distance = 5.0
            distance += self.random.gauss(0.0, 0.004)
            result.append(int(max(50.0, min(5000.0, distance * 1000.0))))
        return result

    def _cardinal(self) -> dict[str, float]:
        return cardinal_distances(sanitize_scan(self.lidar()))

    def front_lidar(self) -> float:
        return self._cardinal()["front"]

    def rear_lidar(self) -> float:
        return self._cardinal()["rear"]

    def left_lidar(self) -> float:
        return self._cardinal()["left"]

    def right_lidar(self) -> float:
        return self._cardinal()["right"]

    def left_front_lidar(self) -> float:
        return self._cardinal()["front_left"]

    def right_front_lidar(self) -> float:
        return self._cardinal()["front_right"]

    def left_rear_lidar(self) -> float:
        scan = sanitize_scan(self.lidar())
        return cardinal_distances(scan)["rear"]

    def right_rear_lidar(self) -> float:
        scan = sanitize_scan(self.lidar())
        return cardinal_distances(scan)["rear"]

    def left_encoder(self) -> float:
        self._update()
        return self.left_distance_m * 1000.0

    def right_encoder(self) -> float:
        self._update()
        return self.right_distance_m * 1000.0

    def gyroscope_z(self) -> float:
        self._update()
        left_mps = wheel_percent_to_mps(self.left_percent)
        right_mps = wheel_percent_to_mps(self.right_percent)
        angular = (right_mps - left_mps) / 0.0956
        return math.degrees(angular) + self.gyro_bias_dps + self.random.gauss(0.0, 0.12)

    def gyroscope_x(self) -> float:
        return self.random.gauss(0.0, 0.04)

    def gyroscope_y(self) -> float:
        return self.random.gauss(0.0, 0.04)

    def accelerometer_x(self) -> float:
        return self.random.gauss(0.0, 0.02)

    def accelerometer_y(self) -> float:
        return self.random.gauss(0.0, 0.02)

    def accelerometer_z(self) -> float:
        return 1.0 + self.random.gauss(0.0, 0.015)

    def battery_state(self) -> int:
        return 3

    def signal_strength(self) -> int:
        return -42

    def temperature(self) -> float:
        return 27.0

    def charge_state(self) -> int:
        return 0

    def tilt(self) -> int:
        return -3

    def servo_input_a(self) -> float:
        return 0.0

    def sound(self, name: str, count: int | None = None) -> None:
        print(f"[MOCK] sound={name} count={count}")

    def sound_until_done(self, name: str) -> None:
        print(f"[MOCK] sound={name}")


class SafeBeagle:
    """속도 제한, 종료 정지, dry-run을 제공하는 안전 래퍼."""

    def __init__(
        self,
        *,
        dry_run: bool = False,
        max_speed: float = 25.0,
        scene: str = "default",
    ) -> None:
        self.dry_run = dry_run
        self.max_speed = abs(float(max_speed))
        self._closed = False
        if dry_run:
            self.robot: Any = MockBeagle(scene=scene)
        else:
            try:
                from roboid import Beagle  # type: ignore
            except ImportError as exc:
                raise RuntimeError("roboid를 불러오지 못했습니다. --dry-run으로 먼저 실행하세요.") from exc
            self.robot = Beagle()
        atexit.register(self.stop)
        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
            if sig is not None:
                try:
                    signal.signal(sig, self._signal_handler)
                except (ValueError, OSError):
                    pass

    def _signal_handler(self, signum: int, frame: object) -> None:
        self.stop()
        raise KeyboardInterrupt

    def __enter__(self) -> "SafeBeagle":
        return self

    def __exit__(self, exc_type, exc, tb) -> None:
        self.stop()

    def wheels(self, left: float, right: float) -> None:
        left = max(-self.max_speed, min(self.max_speed, float(left)))
        right = max(-self.max_speed, min(self.max_speed, float(right)))
        self.robot.wheels(left, right)

    def stop(self) -> None:
        try:
            self.robot.stop()
        except Exception:
            pass

    def close(self) -> None:
        self.stop()
        self._closed = True

    def start_lidar(self) -> None:
        self.robot.start_lidar()

    def wait_until_lidar_ready(self) -> None:
        if hasattr(self.robot, "wait_until_lidar_ready"):
            self.robot.wait_until_lidar_ready()
        else:
            deadline = time.monotonic() + 4.0
            while time.monotonic() < deadline:
                if getattr(self.robot, "is_lidar_ready", lambda: True)():
                    return
                time.sleep(0.05)
            raise TimeoutError("LiDAR 준비 시간이 초과되었습니다.")

    def lidar(self) -> list[float]:
        return sanitize_scan(self.robot.lidar())

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)
