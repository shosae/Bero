import math
from collections import deque
from statistics import pstdev
from dataclasses import dataclass

from bero_perception_imu.resampler import ImuSample


@dataclass(frozen=True, slots=True)
class DetectResult:
    """Detected segment와 소요 시간."""

    seg_buf: list[ImuSample]
    seg_dur: float


class SegmentDetector:
    """Resampling된 샘플로 moving/stopped 판정 및 segment 탐지."""

    def __init__(
        self,
        fs: float,
        acc_high: float,
        acc_low: float,
        std_low: float,
        stop_dur: float,
        stop_speed_threshold: float,
        pre_buf_sec: float,
        pre_buf_trim_sec: float,
    ) -> None:
        # ---------------- Configuration ----------------
        self._acc_high = acc_high
        self._acc_low = acc_low
        self._std_low = std_low
        self._stop_speed_threshold = stop_speed_threshold

        self._pre_buf_size = math.ceil(fs * pre_buf_sec) + 1
        self._pre_buf_trim_count = math.ceil(fs * pre_buf_trim_sec)
        self._stop_count_threshold = math.ceil(stop_dur * fs)

        # ---------------- Runtime Variables ----------------
        self._acc_win = deque(maxlen=max(1, math.ceil(fs * 0.3)))
        self._latest_vel = 0.0
        self._latest_buf = deque(maxlen=self._pre_buf_size)
        self._is_segment_opened = False
        self._stop_count = 0
        self._seg_buf: list[ImuSample] = []

    # ---------------- Public ----------------

    def reset(self) -> None:
        """Detector 상태 초기화."""
        self._acc_win.clear()
        self._latest_vel = 0.0
        self._latest_buf.clear()
        self._is_segment_opened = False
        self._stop_count = 0
        self._seg_buf = []

    def detect(
        self,
        sample: ImuSample,
    ) -> DetectResult | None:
        """Resampling된 샘플로 moving/stopped 판정 및 segment 탐지 후 반환."""
        prev_t = self._latest_buf[-1].t if self._latest_buf else None

        self._latest_buf.append(sample)
        self._acc_win.append(sample.az_lpf)

        std = pstdev(self._acc_win)
        acc_dev = abs(sample.az_lpf)

        is_moving = acc_dev > self._acc_high
        is_stopped = (std < self._std_low) and (acc_dev < self._acc_low)

        if not self._is_segment_opened and not is_moving:
            self._latest_vel = 0.0
        elif prev_t is None:
            self._latest_vel = 0.0
        else:
            dt = sample.t - prev_t
            self._latest_vel += sample.az_lpf * dt

        if (
            self._is_segment_opened
            and is_stopped
            and abs(self._latest_vel) > self._stop_speed_threshold
        ):
            is_stopped = False

        return self._detect_segment(
            sample=sample,
            is_moving=is_moving,
            is_stopped=is_stopped,
        )

    def get_latest_state(self) -> tuple[float, float, float] | None:
        """현재 최신 상태 반환: (time, az_lpf, vel)."""
        if not self._latest_buf:
            return None
        rel_t, az_lpf = self._latest_buf[-1].t, self._latest_buf[-1].az_lpf
        return (
            rel_t,
            az_lpf,
            self._latest_vel,
        )

    # ---------------- Helper methods ----------------

    def _detect_segment(
        self,
        sample: ImuSample,
        is_moving: bool,
        is_stopped: bool,
    ) -> DetectResult | None:
        """moving/stopped 상태 변화로 segment open/close 판단."""
        if not self._is_segment_opened:
            if is_moving:
                self._is_segment_opened = True
                self._stop_count = 0
                self._seg_buf = list(self._latest_buf)
            return None

        self._seg_buf.append(sample)

        if is_stopped:
            self._stop_count += 1
        else:
            self._stop_count = 0

        if self._stop_count < self._stop_count_threshold:
            return None

        seg_dur = self._compute_seg_dur()
        closed_segment = DetectResult(
            seg_buf=list(self._seg_buf),
            seg_dur=seg_dur,
        )

        self._is_segment_opened = False
        self._stop_count = 0
        self._seg_buf = []
        self._latest_vel = 0.0

        return closed_segment

    def _compute_seg_dur(self) -> float:
        """버퍼를 제외한 실제 segment duration 계산."""
        seg_end_idx = len(self._seg_buf) - self._stop_count - 1
        seg_start_idx = min(self._pre_buf_trim_count, seg_end_idx)

        start_t = self._seg_buf[seg_start_idx].t
        end_t = self._seg_buf[seg_end_idx].t
        return end_t - start_t
