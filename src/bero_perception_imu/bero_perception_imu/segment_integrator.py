from dataclasses import dataclass

import numpy as np
from scipy.signal import butter, sosfiltfilt

from bero_perception_imu.resampler import ImuSample


@dataclass(frozen=True, slots=True)
class SegmentResult:
    """Segment의 수직 이동 거리와 소요 시간."""

    dh: float
    dur: float


class SegmentIntegrator:
    """Segment를 적분해 수직 이동 거리 계산."""

    def __init__(
        self,
        fs: float,
        fc: float,
        filtfilt_order: int,
        pre_buf_trim_sec: float,
        stop_dur: float,
        zupt_window: float,
    ) -> None:
        # ---------------- Configuration ----------------
        self.fs = fs
        self.fc = fc
        self.filtfilt_order = filtfilt_order
        self.pre_buf_trim_sec = pre_buf_trim_sec
        self.stop_dur = stop_dur
        self.zupt_window = zupt_window

        if self.filtfilt_order > 0:
            self._sos_fs = butter(
                self.filtfilt_order,
                self.fc,
                fs=self.fs,
                btype='low',
                output='sos',
            )
        else:
            self._sos_fs = None

    # ---------------- Public ----------------

    def integrate(
        self,
        seg_buf: list[ImuSample],
    ) -> float:
        """Segment 필터링 + ZUPT 적분으로 수직 이동 거리 반환."""
        seg_times, seg_az_clipped = zip(*((s.t, s.az_clipped) for s in seg_buf))

        if self._sos_fs is not None:
            try:
                # filtfilt
                seg_filtered = sosfiltfilt(
                    self._sos_fs,
                    np.array(seg_az_clipped, dtype=float),
                )
            except ValueError:
                # 짧은 세그먼트일 때 lpf 사용
                seg_az_lpf = [s.az_lpf for s in seg_buf]
                seg_filtered = np.array(seg_az_lpf, dtype=float)
        else:
            # clipped
            seg_filtered = np.array(seg_az_clipped, dtype=float)

        trimmed_seg_times, trimmed_seg_filtered = self._trim_segment(
            np.asarray(seg_times, dtype=float),
            np.asarray(seg_filtered, dtype=float),
        )

        if len(trimmed_seg_times) <= 1:
            return 0.0

        # 변위 dh 계산
        dh_m = self._integrate_with_zupt(
            trimmed_seg_times.tolist(),
            trimmed_seg_filtered,
            zupt_window=self.zupt_window,
        )

        return dh_m

    # ---------------- Helper methods ----------------

    def _trim_segment(
        self,
        seg_times: np.ndarray,
        seg_filtered: np.ndarray,
    ) -> tuple[np.ndarray, np.ndarray]:
        """Segment 앞뒤 buffer 제거."""
        # 시간 기준 trimming
        trim_start_t = float(seg_times[0] + self.pre_buf_trim_sec)
        trim_end_t = float(seg_times[-1] - (self.stop_dur - self.zupt_window))

        # idx 계산
        start_idx = int(np.searchsorted(seg_times, trim_start_t - 1e-9, side="left"))
        end_idx = int(np.searchsorted(seg_times, trim_end_t + 1e-9, side="right"))

        if trim_start_t >= trim_end_t or end_idx <= start_idx:
            return seg_times[0:0], seg_filtered[0:0]

        return seg_times[start_idx:end_idx], seg_filtered[start_idx:end_idx]

    @staticmethod
    def _integrate_with_zupt(
        seg_times: list[float],
        seg_az_filt: list[float] | np.ndarray,
        zupt_window: float,
    ) -> float:
        """ZUPT 기반 속도 drift 보정 후 높이 적분."""
        n = len(seg_times)
        v = [0.0]

        # 속도 계산
        for i in range(1, n):
            dt = seg_times[i] - seg_times[i - 1]
            avg_acc = 0.5 * (seg_az_filt[i - 1] + seg_az_filt[i])
            v.append(v[-1] + avg_acc * dt)

        # ZUPT 시작 idx 계산
        zupt_start_idx = 0
        for i in range(n - 1, -1, -1):
            if (seg_times[-1] - seg_times[i]) > (zupt_window + 1e-9):
                zupt_start_idx = i + 1
                break

        # drift mean 계산
        drift = float(np.mean(v[zupt_start_idx:])) if zupt_start_idx < n else 0.0

        # 선형적으로 drift 보정
        if 0 < zupt_start_idx < n and abs(drift) > 1e-6:
            start_t = seg_times[0]
            end_t = seg_times[zupt_start_idx]
            span = end_t - start_t
            if span > 1e-9:
                for i in range(zupt_start_idx):
                    alpha = (seg_times[i] - start_t) / span
                    v[i] -= drift * alpha

        # ZUPT window 속도를 0으로 설정
        for i in range(zupt_start_idx, n):
            v[i] = 0.0

        # 변위 계산
        dh_m = 0.0
        for i in range(1, n):
            dt = seg_times[i] - seg_times[i - 1]
            avg_vel = 0.5 * (v[i - 1] + v[i])
            dh_m += avg_vel * dt

        return dh_m
