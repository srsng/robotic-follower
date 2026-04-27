"""轻量性能计时与聚合工具。"""

from __future__ import annotations

import time
from dataclasses import dataclass
from typing import Callable


@dataclass
class _Stat:
    min_v: float
    max_v: float
    sum_v: float
    count: int


class PerfFrame:
    """单帧(或单步)性能计时容器。"""

    def __init__(self, timer: "PerfTimer"):
        self._timer = timer
        self._marks: dict[str, float] = {}
        self._values: dict[str, float | int | str | bool] = {}

    def mark(self, name: str) -> float:
        now = time.monotonic()
        self._marks[name] = now
        return now

    def record(self, name: str, start: str | float):
        if isinstance(start, str):
            begin = self._marks.get(start)
            if begin is None:
                return
        else:
            begin = float(start)
        self._values[name] = max(0.0, time.monotonic() - begin)

    def record_value(self, name: str, value: float | int | str | bool):
        self._values[name] = value

    def flush(
        self,
        *,
        extra: dict[str, float | int | str | bool] | None = None,
        level: str = "debug",
    ):
        payload = dict(self._values)
        if extra:
            payload.update(extra)
        self._timer.emit(payload, level=level)


class PerfTimer:
    """性能日志输出器，支持逐条记录与定期统计。"""

    def __init__(
        self,
        log_fn: Callable[[str, str, str], None],
        *,
        channel: str,
        stats_channel: str,
        aggregate_interval: int = 100,
    ):
        self._log_fn = log_fn
        self._channel = channel
        self._stats_channel = stats_channel
        self._aggregate_interval = max(1, int(aggregate_interval))
        self._row_count = 0
        self._stats: dict[str, _Stat] = {}

    def frame(self) -> PerfFrame:
        return PerfFrame(self)

    def emit(self, values: dict[str, float | int | str | bool], *, level: str = "debug"):
        self._row_count += 1
        self._update_stats(values)
        self._log_fn(level, self._serialize(values), self._channel)
        if self._row_count % self._aggregate_interval == 0:
            self._flush_stats()

    def _update_stats(self, values: dict[str, float | int | str | bool]):
        for key, value in values.items():
            if not self._should_aggregate(key, value):
                continue
            v = float(value)
            st = self._stats.get(key)
            if st is None:
                self._stats[key] = _Stat(min_v=v, max_v=v, sum_v=v, count=1)
            else:
                st.min_v = min(st.min_v, v)
                st.max_v = max(st.max_v, v)
                st.sum_v += v
                st.count += 1

    @staticmethod
    def _should_aggregate(key: str, value: float | int | str | bool) -> bool:
        if key in {"frame_id", "step_id"} or key.endswith("_id"):
            return False
        if isinstance(value, bool):
            return False
        return isinstance(value, (int, float))

    def _flush_stats(self):
        if not self._stats:
            return
        payload: dict[str, float | int] = {}
        for key in sorted(self._stats.keys()):
            st = self._stats[key]
            avg = st.sum_v / max(1, st.count)
            payload[f"{key}_min"] = st.min_v
            payload[f"{key}_max"] = st.max_v
            payload[f"{key}_avg"] = avg
            payload[f"{key}_n"] = st.count
        payload["rows"] = self._row_count
        self._log_fn("info", self._serialize(payload), self._stats_channel)
        self._stats.clear()

    @staticmethod
    def _serialize(values: dict[str, float | int | str | bool]) -> str:
        parts: list[str] = []
        for key, value in values.items():
            if isinstance(value, bool):
                out = "1" if value else "0"
            elif isinstance(value, float):
                out = f"{value:.6f}"
            else:
                out = str(value)
            parts.append(f"{key}={out}")
        return " ".join(parts)
