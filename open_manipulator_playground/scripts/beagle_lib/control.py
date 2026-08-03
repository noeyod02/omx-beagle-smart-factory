from __future__ import annotations

from collections import deque
from dataclasses import dataclass, field
import time
from typing import Deque, Generic, Iterable, TypeVar

from .geometry import clamp

T = TypeVar("T")


@dataclass
class StableValue(Generic[T]):
    """같은 값이 N번 연속일 때만 안정된 값으로 인정합니다."""

    frames: int = 4
    history: Deque[T] = field(init=False)

    def __post_init__(self) -> None:
        if self.frames <= 0:
            raise ValueError("frames must be positive")
        self.history = deque(maxlen=self.frames)

    def update(self, value: T) -> T | None:
        self.history.append(value)
        if len(self.history) == self.frames and all(item == value for item in self.history):
            return value
        return None

    def clear(self) -> None:
        self.history.clear()


@dataclass
class StableBoolean:
    true_frames: int = 4
    false_frames: int = 3
    state: bool = False
    _true_count: int = 0
    _false_count: int = 0

    def update(self, value: bool) -> bool:
        if value:
            self._true_count += 1
            self._false_count = 0
            if self._true_count >= self.true_frames:
                self.state = True
        else:
            self._false_count += 1
            self._true_count = 0
            if self._false_count >= self.false_frames:
                self.state = False
        return self.state


@dataclass
class HysteresisThreshold:
    """low보다 작으면 ON, high보다 크면 OFF가 되는 저거리 위험 판정."""

    low: float
    high: float
    active: bool = False

    def __post_init__(self) -> None:
        if self.low >= self.high:
            raise ValueError("low must be smaller than high")

    def update(self, value: float) -> bool:
        if self.active and value >= self.high:
            self.active = False
        elif not self.active and value <= self.low:
            self.active = True
        return self.active


@dataclass
class Watchdog:
    timeout_s: float = 0.3
    last_feed: float = field(default_factory=time.monotonic)

    def feed(self) -> None:
        self.last_feed = time.monotonic()

    def expired(self, now: float | None = None) -> bool:
        now = time.monotonic() if now is None else now
        return now - self.last_feed > self.timeout_s


@dataclass
class StateTimer:
    state: str = "WAIT"
    started_at: float = field(default_factory=time.monotonic)

    def set(self, new_state: str) -> bool:
        if new_state == self.state:
            return False
        self.state = new_state
        self.started_at = time.monotonic()
        return True

    def elapsed(self) -> float:
        return time.monotonic() - self.started_at


def p_wall_follow(
    measured_mm: float,
    target_mm: float,
    *,
    kp: float,
    base_speed: float,
    follow_side: str = "right",
    dead_band_mm: float = 5.0,
    max_correction: float = 8.0,
) -> tuple[float, float, float]:
    """벽 거리 오차를 좌우 바퀴 보정으로 바꿉니다."""

    error = target_mm - measured_mm
    if abs(error) <= dead_band_mm:
        error = 0.0
    correction = clamp(kp * error, -max_correction, max_correction)
    if follow_side.lower() == "right":
        # 오른쪽 벽에 너무 가까움(error>0) -> 왼쪽으로 회피: 오른쪽 바퀴를 더 빠르게
        left = base_speed - correction
        right = base_speed + correction
    else:
        left = base_speed + correction
        right = base_speed - correction
    return left, right, correction


def first_true_priority(rules: Iterable[tuple[bool, str]]) -> str:
    for condition, state in rules:
        if condition:
            return state
    return "NORMAL"
