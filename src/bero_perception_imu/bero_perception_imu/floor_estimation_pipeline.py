from dataclasses import dataclass

from bero_perception_imu.resampler import ImuResampler
from bero_perception_imu.segment_detector import SegmentDetector
from bero_perception_imu.segment_integrator import SegmentIntegrator, SegmentResult
from bero_perception_imu.floor_tracker import FloorTracker, TrackResult


@dataclass(frozen=True, slots=True)
class PipelineConfig:
    """Floor estimation 파이프라인 설정값."""

    floors: list[int]
    h_L1: float
    h_12: float
    h_norm: float
    acc_clip_abs: float
    raw_fs: float
    fs: float
    fc: float
    filt_order: int
    filtfilt_order: int
    min_seg_dur: float
    min_seg_h: float
    pre_buf_sec: float
    pre_buf_trim_sec: float
    acc_high: float
    acc_low: float
    std_low: float
    stop_speed_threshold: float
    stop_dur: float
    zupt_window: float

    def __post_init__(self) -> None:
        self._validate()

    def _validate(self) -> None:
        if len(self.floors) < 2:
            raise ValueError("floors must contain at least 2 values")
        if len(set(self.floors)) != len(self.floors):
            raise ValueError("floors must not contain duplicates")
        if self.floors != sorted(self.floors):
            raise ValueError("floors must be sorted in ascending order")

        if self.h_L1 <= 0.0:
            raise ValueError("h_L1 must be > 0")
        if self.h_12 <= 0.0:
            raise ValueError("h_12 must be > 0")
        if self.h_norm <= 0.0:
            raise ValueError("h_norm must be > 0")
        if self.acc_clip_abs <= 0.0:
            raise ValueError("acc_clip_abs must be > 0")

        if self.raw_fs <= 0.0:
            raise ValueError("raw_fs must be > 0")
        if self.fs <= 0.0:
            raise ValueError("fs must be > 0")
        if self.fc <= 0.0:
            raise ValueError("fc must be > 0")
        if self.fc >= (0.5 * self.raw_fs):
            raise ValueError("fc must be < raw_fs / 2")
        if self.fc >= (0.5 * self.fs):
            raise ValueError("fc must be < fs / 2")
        if self.filt_order <= 0:
            raise ValueError("filt_order must be > 0")
        if self.filtfilt_order < 0:
            raise ValueError("filtfilt_order must be >= 0")

        if self.min_seg_dur < 0.0:
            raise ValueError("min_seg_dur must be >= 0")
        if self.min_seg_h < 0.0:
            raise ValueError("min_seg_h must be >= 0")
        if self.pre_buf_sec < 0.0:
            raise ValueError("pre_buf_sec must be >= 0")
        if self.pre_buf_trim_sec < 0.0:
            raise ValueError("pre_buf_trim_sec must be >= 0")
        if self.pre_buf_trim_sec > self.pre_buf_sec:
            raise ValueError("pre_buf_trim_sec must be <= pre_buf_sec")
        if self.acc_high <= 0.0:
            raise ValueError("acc_high must be > 0")
        if self.acc_low < 0.0:
            raise ValueError("acc_low must be >= 0")
        if self.acc_low >= self.acc_high:
            raise ValueError("acc_low must be < acc_high")
        if self.std_low < 0.0:
            raise ValueError("std_low must be >= 0")
        if self.stop_speed_threshold <= 0.0:
            raise ValueError("stop_speed_threshold must be > 0")
        if self.stop_dur <= 0.0:
            raise ValueError("stop_dur must be > 0")
        if self.zupt_window < 0.0:
            raise ValueError("zupt_window must be >= 0")
        if self.stop_dur < self.zupt_window:
            raise ValueError("stop_dur must be >= zupt_window")


@dataclass(frozen=True, slots=True)
class PipelineUpdate:
    """Segment 분석 및 층 추정 결과."""

    seg_result: SegmentResult
    track_result: TrackResult | None = None


class FloorEstimationPipeline:
    """IMU Resample -> Segment Detect -> Segment Integrate -> Floor Track으로 구성된 파이프라인."""

    def __init__(self, config: PipelineConfig) -> None:
        # ---------------- Configuration ----------------
        self.config = config

        # ---------------- Runtime Variables ----------------
        self._imu_t0_stamp_sec: float | None = None
        self._resampler = ImuResampler(
            fs=config.fs,
            raw_fs=config.raw_fs,
            fc=config.fc,
            filt_order=config.filt_order,
            baseline=0.0,
            clip_abs=config.acc_clip_abs,
        )
        self._segment_detector = SegmentDetector(
            fs=config.fs,
            acc_high=config.acc_high,
            acc_low=config.acc_low,
            std_low=config.std_low,
            stop_dur=config.stop_dur,
            stop_speed_threshold=config.stop_speed_threshold,
            pre_buf_sec=config.pre_buf_sec,
            pre_buf_trim_sec=config.pre_buf_trim_sec,
        )
        self._segment_integrator = SegmentIntegrator(
            fs=config.fs,
            fc=config.fc,
            filtfilt_order=config.filtfilt_order,
            pre_buf_trim_sec=config.pre_buf_trim_sec,
            stop_dur=config.stop_dur,
            zupt_window=config.zupt_window,
        )
        self._tracker = FloorTracker(
            floors=config.floors,
            h_L1=config.h_L1,
            h_12=config.h_12,
            h_norm=config.h_norm,
            start_floor=config.floors[0],
            target_floor=config.floors[1],
        )

    # ---------------- Public ----------------

    def initialize_goal(
        self,
        *,
        baseline: float,
        start_floor: int,
        target_floor: int,
    ) -> None:
        """Goal 시작 전 파이프라인 상태 초기화."""
        self._imu_t0_stamp_sec = None

        self._resampler.reset(baseline=baseline)
        self._segment_detector.reset()
        self._tracker.reset(
            start_floor=start_floor,
            target_floor=target_floor,
        )

    def push_sample(self, stamp_sec: float, az_raw: float) -> None:
        """IMU data를 첫 번째 샘플과의 상대적인 시간을 계산하여 파이프라인에 추가."""
        if self._imu_t0_stamp_sec is None:
            self._imu_t0_stamp_sec = stamp_sec

        rel_t = stamp_sec - self._imu_t0_stamp_sec
        self._resampler.push_sample(rel_t, az_raw)

    def tick(self) -> list[PipelineUpdate]:
        """Pipeline 한 사이클."""
        updates: list[PipelineUpdate] = []

        for sample in self._resampler.resample():
            segment = self._segment_detector.detect(sample)
            if segment is None:
                continue
            seg_buf = segment.seg_buf
            seg_dur = segment.seg_dur

            # ignore: dur < min_seg_dur
            if seg_dur < self.config.min_seg_dur:
                continue

            dh = self._segment_integrator.integrate(seg_buf)

            seg_result = SegmentResult(dh=dh, dur=seg_dur)

            # ignore: dh < min_seg_h
            if abs(dh) < self.config.min_seg_h:
                updates.append(
                    PipelineUpdate(
                        seg_result=seg_result,
                    )
                )
                continue

            # 층 추정
            track_result = self._tracker.track(dh)
            updates.append(
                PipelineUpdate(
                    seg_result=seg_result,
                    track_result=track_result,
                )
            )

        return updates

    def get_current_floor(self) -> int:
        """추정된 현재 층 반환."""
        return self._tracker.get_current_floor()

    def get_viz_data(self) -> tuple[float, float, float] | None:
        """시각화용 현재 상태 반환: (time, az_lpf, vel)."""
        return self._segment_detector.get_latest_state()
