from dataclasses import dataclass
from enum import Enum


class TrackDecision(Enum):
    """층 추정 결과에 따른 판단."""

    GETOFF = 'getoff'
    STAY = 'stay'


@dataclass(frozen=True, slots=True)
class TrackResult:
    """층 추정 결과."""

    decision: TrackDecision
    current_floor: int
    h_diff: float


class FloorTracker:
    def __init__(
        self,
        floors: list[int],
        h_L1: float,
        h_12: float,
        h_norm: float,
        start_floor: int,
        target_floor: int,
    ) -> None:
        self.floors = floors
        self.h_L1 = h_L1
        self.h_12 = h_12
        self.h_norm = h_norm
        self.reset(start_floor, target_floor)

    # ---------------- Public ----------------

    def reset(self, start_floor: int, target_floor: int) -> None:
        """시작층과 목표층으로 상태 초기화."""
        if start_floor not in self.floors:
            raise ValueError(f"start_floor {start_floor} is not in : {self.floors}")
        if target_floor not in self.floors:
            raise ValueError(f"target_floor {target_floor} is not in : {self.floors}")

        self._target_floor = target_floor
        self._current_floor = start_floor
        self._floor_heights = self._build_floor_heights(start_floor)
        self._total_h = 0.0

    def track(self, dh: float) -> TrackResult:
        """누적 높이에서 가장 가까운 층으로 추정."""
        self._total_h += dh

        estimated_floor, h_diff = min(
            (
                (floor, abs(self._total_h - height))
                for floor, height in self._floor_heights.items()
            ),
            key=lambda item: item[1],
        )

        self._current_floor = estimated_floor
        self._total_h = self._floor_heights[estimated_floor]

        if estimated_floor != self._target_floor:
            decision = TrackDecision.STAY
        else:
            decision = TrackDecision.GETOFF

        return TrackResult(
            decision=decision,
            current_floor=estimated_floor,
            h_diff=h_diff,
        )

    def get_current_floor(self) -> int:
        """추정된 현재 층 반환."""
        return self._current_floor

    # ---------------- Helper methods ----------------

    def _build_floor_heights(self, start_floor: int) -> dict[int, float]:
        """시작층을 기준으로 모든 층의 높이 계산."""
        idx = self.floors.index(start_floor)
        heights = {start_floor: 0.0}

        up = self.floors[idx:]
        for prev, cur in zip(up[:-1], up[1:]):
            heights[cur] = heights[prev] + self._seg_h(prev, cur)

        down = self.floors[idx::-1]
        for prev, cur in zip(down[:-1], down[1:]):
            heights[cur] = heights[prev] - self._seg_h(prev, cur)

        return heights

    def _seg_h(self, floor_a: int, floor_b: int) -> float:
        """층간 높이 반환."""
        pair = {floor_a, floor_b}
        if pair == {0, 1}:
            return self.h_L1
        if pair == {1, 2}:
            return self.h_12
        return self.h_norm
