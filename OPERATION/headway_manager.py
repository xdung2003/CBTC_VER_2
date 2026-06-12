from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Literal


DispatchDecision = Literal["HOLD", "RELEASE_DISPATCH"]


@dataclass(frozen=True)
class HeadwayDecision:
    train_id: str
    decision: DispatchDecision
    reason: str
    target_headway_s: float
    planned_dispatch_time_s: float | None
    dispatch_delay_s: float


@dataclass
class HeadwayStats:
    target_headway_s: float = 0.0
    actual_headways_s: List[float] = field(default_factory=list)
    actual_headway_pairs: List[Dict[str, Any]] = field(default_factory=list)
    dispatch_delays_s: List[float] = field(default_factory=list)
    dispatch_times_s: Dict[str, float] = field(default_factory=dict)
    release_times_s: Dict[str, float] = field(default_factory=dict)

    @property
    def min_actual_headway_s(self) -> float | None:
        return min(self.actual_headways_s) if self.actual_headways_s else None

    @property
    def avg_actual_headway_s(self) -> float | None:
        if not self.actual_headways_s:
            return None
        return sum(self.actual_headways_s) / len(self.actual_headways_s)

    @property
    def max_actual_headway_s(self) -> float | None:
        return max(self.actual_headways_s) if self.actual_headways_s else None

    @property
    def avg_dispatch_delay_s(self) -> float | None:
        if not self.dispatch_delays_s:
            return None
        return sum(self.dispatch_delays_s) / len(self.dispatch_delays_s)


class HeadwayManager:
    """Operational dispatch regulator.

    This manager only decides whether a staged train may be released from the
    dispatch gate. It never grants authority, never computes EOA and never
    overrides ATP.
    """

    def __init__(
        self,
        mode: str = "fixed",
        target_headway_s: float = 0.0,
    ):
        self.mode = "fixed"
        self.target_headway_s = max(0.0, float(target_headway_s))
        self.released_train_ids: set[str] = set()
        self.stats = HeadwayStats(target_headway_s=self.nominal_target_headway_s())

    @classmethod
    def from_scenario(cls, scenario: Dict[str, Any]) -> "HeadwayManager":
        cfg = scenario.get("headway", {}) or {}
        if not isinstance(cfg, dict):
            cfg = {}
        return cls(
            mode="fixed",
            target_headway_s=float(cfg.get("target_headway_s", cfg.get("fixed_headway_s", 0.0))),
        )

    def nominal_target_headway_s(self) -> float:
        return self.target_headway_s

    def enabled(self) -> bool:
        return self.nominal_target_headway_s() > 0.0

    def _planned_time_for_sequence(self, sequence_index: int) -> float | None:
        target = self.nominal_target_headway_s()
        return sequence_index * target if target > 0.0 else None

    def _effective_target_headway_s(self, tsr_active: bool) -> float:
        return self.nominal_target_headway_s()

    def decide(
        self,
        train: Any,
        now_s: float,
        dispatched_front_pos_m: float | None = None,
        tsr_active: bool = False,
    ) -> HeadwayDecision:
        if train.id in self.released_train_ids or not self.enabled():
            return HeadwayDecision(train.id, "RELEASE_DISPATCH", "HEADWAY_DISABLED_OR_ALREADY_RELEASED", 0.0, None, 0.0)

        sequence_index = len(self.released_train_ids)
        planned_time = None
        if planned_time is None:
            planned_time = self._planned_time_for_sequence(sequence_index)
        target = self._effective_target_headway_s(tsr_active)
        self.stats.target_headway_s = target

        if planned_time is not None and now_s + 1e-9 < planned_time:
            return HeadwayDecision(
                train.id,
                "HOLD",
                "HEADWAY_NOT_DUE",
                target,
                planned_time,
                0.0,
            )

        last_release = max(self.stats.release_times_s.values(), default=None)
        if last_release is not None and target > 0.0 and now_s - last_release + 1e-9 < target:
            return HeadwayDecision(
                train.id,
                "HOLD",
                "TARGET_HEADWAY_ACTIVE",
                target,
                planned_time,
                0.0,
            )

        delay = max(0.0, now_s - planned_time) if planned_time is not None else 0.0
        self.released_train_ids.add(train.id)
        self.stats.release_times_s[train.id] = now_s
        self.stats.dispatch_delays_s.append(delay)
        return HeadwayDecision(train.id, "RELEASE_DISPATCH", "HEADWAY_RELEASED", target, planned_time, delay)

    def mark_actual_dispatch(self, train_id: str, now_s: float):
        if train_id in self.stats.dispatch_times_s:
            return
        previous_train_id = None
        previous = None
        if self.stats.dispatch_times_s:
            previous_train_id, previous = max(self.stats.dispatch_times_s.items(), key=lambda item: item[1])
        self.stats.dispatch_times_s[train_id] = now_s
        if previous is not None:
            actual_headway_s = max(0.0, now_s - previous)
            self.stats.actual_headways_s.append(actual_headway_s)
            self.stats.actual_headway_pairs.append(
                {
                    "front_train_id": previous_train_id,
                    "following_train_id": train_id,
                    "front_dispatch_time_s": previous,
                    "following_dispatch_time_s": now_s,
                    "actual_headway_s": actual_headway_s,
                }
            )

    def snapshot(self) -> Dict[str, Any]:
        return {
            "mode": self.mode,
            "target_headway_s": self.stats.target_headway_s,
            "actual_headways_s": list(self.stats.actual_headways_s),
            "actual_headway_pairs": [dict(pair) for pair in self.stats.actual_headway_pairs],
            "min_actual_headway_s": self.stats.min_actual_headway_s,
            "avg_actual_headway_s": self.stats.avg_actual_headway_s,
            "max_actual_headway_s": self.stats.max_actual_headway_s,
            "dispatch_delays_s": list(self.stats.dispatch_delays_s),
            "avg_dispatch_delay_s": self.stats.avg_dispatch_delay_s,
            "dispatch_times_s": dict(self.stats.dispatch_times_s),
            "release_times_s": dict(self.stats.release_times_s),
        }

