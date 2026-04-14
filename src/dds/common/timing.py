"""Shared DDS manager timing and freshness helpers."""

from __future__ import annotations

from dataclasses import dataclass
import math

from runtime_logging import get_logger

from .lowcmd_types import LowCmdCache


LOGGER = get_logger("dds.timing")


@dataclass
class CadenceTracker:
    """Track rolling lowstate cadence diagnostics."""

    label: str
    window_start_time: float | None = None
    publish_count: int = 0

    def record(self, t: float, expected_hz: float, interval: int, warn_ratio: float) -> None:
        if interval <= 0:
            return
        if self.window_start_time is None:
            self.window_start_time = t
            self.publish_count = 1
            return

        self.publish_count += 1
        if self.publish_count < interval:
            return

        elapsed = t - self.window_start_time
        publishes = self.publish_count
        observed_hz = 0.0 if elapsed <= 0.0 else (publishes - 1) / elapsed
        relative_error = 0.0 if expected_hz <= 0.0 else abs(observed_hz - expected_hz) / expected_hz
        log_method = LOGGER.warning if relative_error > warn_ratio else LOGGER.info
        log_method(
            "lowstate cadence check (%s): observed=%.3fHz expected=%.3fHz publishes=%s "
            "window_dt=%.3fs rel_error=%.3f%%",
            self.label,
            observed_hz,
            expected_hz,
            publishes,
            elapsed,
            relative_error * 100.0,
        )

        self.window_start_time = t
        self.publish_count = 1


def compute_publish_period(publish_hz: float) -> float:
    """Convert a positive publish rate into a simulation-time period."""
    if publish_hz <= 0.0:
        raise ValueError(f"lowstate publish rate must be positive, got {publish_hz}")
    if not math.isfinite(publish_hz):
        raise ValueError(f"lowstate publish rate must be finite, got {publish_hz}")
    return 1.0 / publish_hz


def is_fresh(now_monotonic: float, cached: LowCmdCache | None, timeout_seconds: float) -> bool:
    """Return whether a cached lowcmd sample should be treated as fresh."""
    if cached is None:
        return False
    if timeout_seconds <= 0.0:
        return True
    return (now_monotonic - cached.received_at_monotonic) <= timeout_seconds
