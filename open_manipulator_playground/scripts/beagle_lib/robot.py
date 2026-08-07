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
            self._require_connection()
        atexit.register(self.stop)
        for sig in (getattr(signal, "SIGINT", None), getattr(signal, "SIGTERM", None)):
            if sig is not None:
                try:
                    signal.signal(sig, self._signal_handler)
                except (ValueError, OSError):
                    pass

    def _require_connection(self, timeout: float = 120.0) -> None:
        """로봇과 실제로 이어졌는지 확인하고, 아니면 여기서 멈춘다.

        Beagle()은 연결에 실패해도 예외를 던지지 않는다. 연결되지 않은 객체를
        그대로 돌려주고, 그 객체는 바퀴 명령을 조용히 삼키며 모든 센서를 0으로
        읽는다. 그래서 연결이 없을 때의 증상이 "잘 도는 로봇"과 구분되지
        않는다. 주행은 거리를 채우지 못한 채 시간 만료로 끝나고, 그것이
        "도착"으로 보고된다. 2026-08-05에 이렇게 세 번을 제자리에서 달렸다.

        기다리는 시간이 긴 것은 실수가 아니다. 같은 날 측정에서 동글이 로봇을
        잡는 데 약 50초가 걸렸다. 그 사이 is_connected()는 계속 False이고
        Beagle()은 이미 돌아와 있으므로, 성급한 제한은 멀쩡한 로봇을 고장으로
        보고한다. 잡히면 즉시 빠져나가므로 연결이 빠른 날에는 비용이 없다.

        진행 상황을 찍는 것도 그래서다. 아무 말 없이 1분을 서 있으면 그것은
        멈춘 노드와 구분되지 않는다.
        """
        deadline = time.time() + timeout
        announced = 0.0
        started = time.time()
        while time.time() < deadline:
            if self.robot.is_connected():
                return
            waited = time.time() - started
            if waited - announced >= 10.0:
                announced = waited
                print(f"[비글] 연결을 기다리는 중... {waited:.0f}s", flush=True)
            time.sleep(0.2)
        raise RuntimeError(
            f'{timeout:.0f}초 안에 비글에 연결되지 못했습니다. 동글은 응답하지만 '
            '로봇과의 링크가 없습니다 - 본체 전원과 배터리, 그리고 이 동글에 '
            '페어링된 기체인지 확인하세요. 로봇 없이 순서만 확인하려면 '
            'dry_run:=true.'
        )

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
        # roboid는 자체 스레드로 로봇과 통신하며, 그 스레드는 데몬이 아니다.
        # stop()은 바퀴만 세울 뿐 스레드를 정리하지 않으므로, dispose()를
        # 부르지 않으면 Ctrl-C를 눌러도 프로세스가 끝나지 않는다.
        # (상위 course 패키지에는 없는 호출이다.)
        if not self.dry_run:
            try:
                import roboid  # type: ignore

                roboid.dispose()
            except Exception:
                pass
        self._closed = True

    def start_lidar(self) -> None:
        self.robot.start_lidar()

    def wait_until_lidar_ready(self, timeout: float = 8.0) -> None:
        """라이다가 회전 속도에 오를 때까지, 단 정해진 시간만 기다린다.

        roboid의 wait_until_lidar_ready()는 시간 제한이 없어서, 라이다가 돌지
        않으면 영원히 돌아오지 않는다. 호출한 쪽에서는 그것이 노드가 멈춘
        것과 구분되지 않으므로, is_lidar_ready()를 직접 폴링해서 기다림에
        끝을 둔다. 상위 course 패키지와 다른 점이다.
        """
        ready = getattr(self.robot, "is_lidar_ready", None)
        if ready is None:
            self.robot.wait_until_lidar_ready()
            return

        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if ready():
                return
            time.sleep(0.05)
        raise TimeoutError(f"LiDAR 준비 시간 {timeout:.0f}초가 초과되었습니다.")

    def lidar(self) -> list[float]:
        return sanitize_scan(self.robot.lidar())

    def __getattr__(self, name: str) -> Any:
        return getattr(self.robot, name)
