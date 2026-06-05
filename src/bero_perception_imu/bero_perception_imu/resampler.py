import math
import numpy as np

from bisect import bisect_left
from collections import deque
from dataclasses import dataclass
from scipy.signal import butter, sosfilt, sosfilt_zi


@dataclass(frozen=True, slots=True)
class ImuSample:
    """Resampling된 IMU 샘플."""

    t: float
    az_clipped: float
    az_lpf: float


class ImuResampler:
    """Raw IMU 샘플 전처리 및 resampling."""

    def __init__(
        self,
        fs: float,
        raw_fs: float,
        fc: float,
        filt_order: int,
        baseline: float,
        clip_abs: float,
    ) -> None:
        # ---------------- Configuration ----------------
        self.fs = fs
        self.raw_fs = raw_fs
        self.fc = fc
        self.filt_order = filt_order
        self.baseline = baseline
        self._clip_abs = clip_abs

        self.dt = 1.0 / fs

        self._sos_raw_fs = butter(
            self.filt_order,
            self.fc,
            fs=self.raw_fs,
            btype='low',
            output='sos',
        )
        self._sos_zi = sosfilt_zi(self._sos_raw_fs) * 0.0

        # ---------------- Runtime Variables ----------------
        self._median_win = deque(maxlen=5)
        self._raw_time_buf = []
        self._raw_az_clipped_buf = []
        self._raw_az_lpf_buf = []
        self._next_bucket_idx = None

    # ---------------- Public ----------------

    def reset(self, baseline: float | None = None) -> None:
        """상태 초기화 및 필요 시 baseline 갱신."""
        if baseline is not None:
            self.baseline = baseline

        self._median_win.clear()
        self._sos_zi = sosfilt_zi(self._sos_raw_fs) * 0.0
        self._raw_time_buf.clear()
        self._raw_az_clipped_buf.clear()
        self._raw_az_lpf_buf.clear()
        self._next_bucket_idx = None

    def push_sample(self, rel_t: float, az_raw: float) -> None:
        """Raw IMU 샘플을 입력받아 LPF 통과 후 버퍼에 저장."""
        if rel_t < 0.0:
            return

        if self._raw_time_buf and rel_t < self._raw_time_buf[-1]:
            return

        az_offset_removed = az_raw - self.baseline

        self._median_win.append(az_offset_removed)
        if len(self._median_win) == self._median_win.maxlen:
            az_median_filtered = float(np.median(self._median_win))
        else:
            az_median_filtered = az_offset_removed

        az_clipped = float(np.clip(az_median_filtered, -self._clip_abs, self._clip_abs))

        az_lpf_arr, self._sos_zi = sosfilt(
            self._sos_raw_fs,
            [az_clipped],
            zi=self._sos_zi,
        )
        az_lpf = float(az_lpf_arr[0])

        self._raw_time_buf.append(rel_t)
        self._raw_az_clipped_buf.append(az_clipped)
        self._raw_az_lpf_buf.append(az_lpf)

        if self._next_bucket_idx is None:
            self._next_bucket_idx = math.floor(rel_t / self.dt)

    def resample(self) -> list[ImuSample]:
        """현재 시점까지의 raw IMU 샘플들을 사용해 resampling된 샘플 생성."""
        samples: list[ImuSample] = []

        if self._next_bucket_idx is None:
            return samples

        latest_t = self._raw_time_buf[-1]

        while True:
            bucket_start_t = self._next_bucket_idx * self.dt
            bucket_center_t = bucket_start_t + (0.5 * self.dt)
            bucket_end_t = bucket_start_t + self.dt

            if bucket_end_t > (latest_t + 1e-9):
                break

            sample = self._resample_bucket(
                bucket_start_t,
                bucket_center_t,
                bucket_end_t,
            )
            if sample is None:
                self._next_bucket_idx += 1
                continue

            az_clipped, az_lpf = sample
            samples.append(ImuSample(bucket_center_t, az_clipped, az_lpf))
            self._next_bucket_idx += 1

        self._trim_raw_buffers()
        return samples

    # ---------------- Helper methods ----------------

    def _resample_bucket(
        self,
        bucket_start_t: float,
        bucket_center_t: float,
        bucket_end_t: float,
    ) -> tuple[float, float] | None:
        """Bucket 대표 샘플 반환."""
        sample_idx_range = self._find_sample_idx_in_bucket(bucket_start_t, bucket_end_t)
        if sample_idx_range is not None:
            az_clipped = self._mean_in_range(
                self._raw_az_clipped_buf,
                sample_idx_range,
            )
            az_lpf = self._mean_in_range(
                self._raw_az_lpf_buf,
                sample_idx_range,
            )
            return (az_clipped, az_lpf)

        anchor = self._find_anchor(bucket_center_t)
        if anchor is None:
            return None

        az_clipped = self._interpolate(
            bucket_center_t,
            self._raw_az_clipped_buf,
            anchor,
        )
        az_lpf = self._interpolate(
            bucket_center_t,
            self._raw_az_lpf_buf,
            anchor,
        )
        return (az_clipped, az_lpf)

    def _find_sample_idx_in_bucket(
        self,
        start_t: float,
        end_t: float,
    ) -> tuple[int, int] | None:
        """[start_t, end_t) 구간에 포함되는 인덱스 범위 반환."""
        start_idx = bisect_left(self._raw_time_buf, start_t)
        end_idx = bisect_left(self._raw_time_buf, end_t)

        if start_idx >= end_idx:
            return None
        return (start_idx, end_idx)

    @staticmethod
    def _mean_in_range(values: list[float], idx_range: tuple[int, int]) -> float:
        """지정된 인덱스 범위의 평균 계산."""
        start_idx, end_idx = idx_range
        bucket_values = values[start_idx:end_idx]
        sample_count = end_idx - start_idx
        return float(sum(bucket_values) / sample_count)

    def _find_anchor(self, target_t: float) -> tuple[int, int] | None:
        """target_t 기준으로 선형 보간 anchor 인덱스 쌍 반환."""
        n = len(self._raw_time_buf)
        if n < 2:
            return None

        idx = bisect_left(self._raw_time_buf, target_t)
        if idx <= 0:
            return (0, 1)
        if idx >= n:
            return (n - 2, n - 1)
        return (idx - 1, idx)

    def _interpolate(
        self,
        target_t: float,
        values: list[float],
        anchor: tuple[int, int],
    ) -> float:
        """Raw timestamp 축에서 target_t 시점 값 선형 보간."""
        left_idx, right_idx = anchor
        t0 = self._raw_time_buf[left_idx]
        t1 = self._raw_time_buf[right_idx]

        v0 = values[left_idx]
        v1 = values[right_idx]

        if abs(t1 - t0) < 1e-9:
            return v1

        alpha = (target_t - t0) / (t1 - t0)
        alpha = max(0.0, min(1.0, alpha))
        return v0 + (v1 - v0) * alpha

    def _trim_raw_buffers(self) -> None:
        """처리 완료된 raw 버퍼에서 interpolate에 사용할 데이터 하나를 제외하고 정리."""
        if len(self._raw_time_buf) < 2:
            return

        keep_from = max(0, bisect_left(self._raw_time_buf, self._next_bucket_idx * self.dt) - 1)

        self._raw_time_buf = self._raw_time_buf[keep_from:]
        self._raw_az_clipped_buf = self._raw_az_clipped_buf[keep_from:]
        self._raw_az_lpf_buf = self._raw_az_lpf_buf[keep_from:]
