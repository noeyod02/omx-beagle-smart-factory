from __future__ import annotations

import math
from statistics import median
from typing import Iterable, Sequence

import numpy as np

INVALID_SENTINEL = 0xFFFF


def clean_distance(
    value: int | float | None,
    *,
    min_mm: int = 50,
    max_mm: int = 5000,
    decode_low_byte: bool = True,
) -> float:
    """Beagle LiDAR 한 값을 유효 거리(mm) 또는 inf로 바꿉니다."""

    try:
        raw = int(value)  # type: ignore[arg-type]
    except (TypeError, ValueError, OverflowError):
        return math.inf
    if raw == INVALID_SENTINEL or raw <= 0:
        return math.inf
    if decode_low_byte and ((raw & 0xFF00) == 0xFF00 or raw < min_mm):
        decoded = raw & 0x00FF
        if min_mm <= decoded <= max_mm:
            raw = decoded
    return float(raw) if min_mm <= raw <= max_mm else math.inf


def sanitize_scan(
    raw: Sequence[int | float],
    *,
    min_mm: int = 50,
    max_mm: int = 5000,
    carry_previous_for_ffff: bool = False,
) -> list[float]:
    cleaned: list[float] = []
    previous = math.inf
    for value in raw:
        distance = clean_distance(value, min_mm=min_mm, max_mm=max_mm)
        if not math.isfinite(distance) and carry_previous_for_ffff:
            try:
                is_ffff = int(value) == INVALID_SENTINEL
            except (TypeError, ValueError):
                is_ffff = False
            if is_ffff:
                distance = previous
        cleaned.append(distance)
        if math.isfinite(distance):
            previous = distance
    return cleaned


def median_filter_circular(scan_mm: Sequence[float], window: int = 5) -> list[float]:
    """원형 배열을 보존하는 간단한 중앙값 필터."""

    if window <= 1 or not scan_mm:
        return [float(v) for v in scan_mm]
    if window % 2 == 0:
        window += 1
    n = len(scan_mm)
    half = window // 2
    output: list[float] = []
    for index in range(n):
        values = [float(scan_mm[(index + offset) % n]) for offset in range(-half, half + 1)]
        finite = [v for v in values if math.isfinite(v)]
        output.append(float(median(finite)) if finite else math.inf)
    return output


def angle_to_index(angle_deg: float, length: int) -> int:
    if length <= 0:
        raise ValueError("length must be positive")
    return int(round((angle_deg % 360.0) / 360.0 * length)) % length


def sector_values(
    scan_mm: Sequence[float],
    center_deg: float,
    half_width_deg: float = 8.0,
) -> list[float]:
    if not scan_mm:
        return []
    n = len(scan_mm)
    center = angle_to_index(center_deg, n)
    half = max(0, int(round(half_width_deg / 360.0 * n)))
    values: list[float] = []
    for offset in range(-half, half + 1):
        value = float(scan_mm[(center + offset) % n])
        if math.isfinite(value) and value > 0.0:
            values.append(value)
    return values


def representative_distance(
    scan_mm: Sequence[float],
    center_deg: float,
    half_width_deg: float = 8.0,
    *,
    mode: str = "median",
    percentile: float = 20.0,
    default: float = math.inf,
) -> float:
    values = sector_values(scan_mm, center_deg, half_width_deg)
    if not values:
        return default
    mode = mode.lower()
    if mode == "min":
        return min(values)
    if mode == "mean":
        return sum(values) / len(values)
    if mode in {"percentile", "quantile"}:
        return float(np.percentile(np.asarray(values), percentile))
    return float(median(values))


def cardinal_distances(scan_mm: Sequence[float], mode: str = "median") -> dict[str, float]:
    return {
        "front": representative_distance(scan_mm, 0, 9, mode=mode),
        "front_left": representative_distance(scan_mm, 35, 12, mode=mode),
        "left": representative_distance(scan_mm, 90, 9, mode=mode),
        "rear": representative_distance(scan_mm, 180, 9, mode=mode),
        "right": representative_distance(scan_mm, 270, 9, mode=mode),
        "front_right": representative_distance(scan_mm, 325, 12, mode=mode),
    }


def choose_open_cardinal(scan_mm: Sequence[float]) -> tuple[str, float]:
    candidates = {
        "front": representative_distance(scan_mm, 0, 15),
        "left": representative_distance(scan_mm, 90, 15),
        "rear": representative_distance(scan_mm, 180, 15),
        "right": representative_distance(scan_mm, 270, 15),
    }
    finite = {name: value for name, value in candidates.items() if math.isfinite(value)}
    if not finite:
        return "none", math.inf
    return max(finite.items(), key=lambda item: item[1])


def detect_openings(
    scan_mm: Sequence[float],
    *,
    open_mm: float = 700.0,
    min_width_deg: float = 12.0,
) -> list[dict[str, float]]:
    """연속된 열린 각도 구간을 찾습니다.

    반환 항목: start_deg, end_deg, center_deg, width_deg, median_mm, score
    """

    if not scan_mm:
        return []
    n = len(scan_mm)
    open_flags = [math.isfinite(v) and float(v) >= open_mm for v in scan_mm]
    # 0도 경계 처리를 위해 배열을 두 번 이어 첫 번째 주기 안의 구간만 수집합니다.
    runs: list[tuple[int, int]] = []
    start: int | None = None
    for index in range(2 * n):
        flag = open_flags[index % n]
        if flag and start is None:
            start = index
        if (not flag or index == 2 * n - 1) and start is not None:
            end = index - 1 if not flag else index
            if start < n:
                runs.append((start, min(end, start + n - 1)))
            start = None
    # 0도 경계를 가로지르는 중복 구간 제거
    normalized: list[tuple[int, int]] = []
    seen: set[tuple[int, int]] = set()
    for start_idx, end_idx in runs:
        width = end_idx - start_idx + 1
        if width <= 0:
            continue
        key = (start_idx % n, width)
        if key not in seen:
            seen.add(key)
            normalized.append((start_idx, end_idx))

    output: list[dict[str, float]] = []
    step = 360.0 / n
    for start_idx, end_idx in normalized:
        width_samples = end_idx - start_idx + 1
        width_deg = width_samples * step
        if width_deg < min_width_deg:
            continue
        values = [float(scan_mm[index % n]) for index in range(start_idx, end_idx + 1)]
        values = [value for value in values if math.isfinite(value)]
        if not values:
            continue
        center_index = (start_idx + end_idx) / 2.0
        median_mm = float(median(values))
        center_deg = (center_index * step) % 360.0
        output.append(
            {
                "start_deg": (start_idx * step) % 360.0,
                "end_deg": (end_idx * step) % 360.0,
                "center_deg": center_deg,
                "width_deg": width_deg,
                "median_mm": median_mm,
                "score": median_mm * width_deg,
            }
        )
    return sorted(output, key=lambda item: item["score"], reverse=True)


def valid_fraction(scan_mm: Iterable[float]) -> float:
    values = list(scan_mm)
    if not values:
        return 0.0
    return sum(math.isfinite(float(v)) for v in values) / len(values)
