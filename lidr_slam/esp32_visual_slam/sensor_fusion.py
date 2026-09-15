"""VL53L1X single-ray ToF sensor fusion and metric scale estimation module.

SCIENTIFIC NOTE & DESIGN PRINCIPLES:
-----------------------------------
A monocular camera observes 3D world points up to an unknown projective scale factor:
    x_c ~ R * X_w + t
where ||t|| = 1.0 (arbitrary unit length).

The VL53L1X sensor emits a single infrared photon cone along the camera's optical axis
(typically 27° field of view, aligned with the forward Z-axis).
It provides a single scalar distance measurement:
    d_tof = ||X_obstacle||_z

This module uses the ToF measurement:
1. To estimate metric scale for forward translation steps.
2. To validate and calibrate triangulated feature depths located within the central optical cone.
3. As a forward collision/obstacle distance monitor.
4. As an extensible base architecture designed for future EKF / IMU / multi-ToF integration.
"""

from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Optional, List, Tuple
import numpy as np

from config.camera_config import SLAMConfig


@dataclass
class ToFMeasurement:
    """Encapsulates a single-ray ToF distance measurement."""
    raw_distance_mm: int
    distance_m: float
    is_valid: bool
    timestamp: float


@dataclass
class FusionState:
    """Current output state from the sensor fusion pipeline."""
    estimated_metric_scale: float
    filtered_distance_m: float
    forward_obstacle_distance_m: Optional[float]
    scale_confidence: float  # [0.0, 1.0]
    is_valid_reading: bool


class SensorFusionBase(ABC):
    """Abstract base class for SLAM sensor fusion architectures.
    
    Subclasses can implement Extended Kalman Filter (EKF), factor graphs,
    or IMU-ToF multi-sensor pre-integration.
    """

    @abstractmethod
    def update(
        self,
        t_rel_unit: np.ndarray,
        raw_tof_mm: int,
        timestamp: float,
        central_feature_depths: Optional[List[float]] = None
    ) -> FusionState:
        """Update sensor fusion state with visual motion and ToF distance."""
        pass


class SingleRayToFFusion(SensorFusionBase):
    """Sensor fusion combining monocular visual odometry with a single-ray VL53L1X sensor."""

    def __init__(self, config: Optional[SLAMConfig] = None):
        self.config = config or SLAMConfig()
        self.min_dist_m = self.config.tof_min_distance_mm / 1000.0
        self.max_dist_m = self.config.tof_max_distance_mm / 1000.0
        self.ema_alpha = self.config.tof_ema_alpha
        self.default_scale = self.config.default_step_scale

        # Internal state
        self._prev_distance_m: Optional[float] = None
        self._filtered_distance_m: Optional[float] = None
        self._current_scale: float = self.default_scale
        self._scale_confidence: float = 0.5

    def filter_tof_reading(self, raw_mm: int, timestamp: float) -> ToFMeasurement:
        """Validates and filters raw ToF readings against sensor operating limits."""
        dist_m = raw_mm / 1000.0
        is_valid = (self.min_dist_m <= dist_m <= self.max_dist_m)

        if is_valid:
            if self._filtered_distance_m is None:
                self._filtered_distance_m = dist_m
            else:
                # Exponential Moving Average filter
                self._filtered_distance_m = (
                    self.ema_alpha * dist_m + (1.0 - self.ema_alpha) * self._filtered_distance_m
                )
        return ToFMeasurement(
            raw_distance_mm=raw_mm,
            distance_m=dist_m,
            is_valid=is_valid,
            timestamp=timestamp
        )

    def update(
        self,
        t_rel_unit: np.ndarray,
        raw_tof_mm: int,
        timestamp: float,
        central_feature_depths: Optional[List[float]] = None
    ) -> FusionState:
        """Estimates metric translation scale and forward obstacle depth.
        
        Args:
            t_rel_unit: Relative translation unit vector (3, 1) from recoverPose.
            raw_tof_mm: VL53L1X range in millimeters.
            timestamp: Frame receive timestamp.
            central_feature_depths: Depths of 3D points near the optical center.
        """
        meas = self.filter_tof_reading(raw_tof_mm, timestamp)

        if not meas.is_valid or not self.config.enable_tof_scale_constraint:
            # Fallback to current smoothed scale or default step size
            return FusionState(
                estimated_metric_scale=self._current_scale,
                filtered_distance_m=self._filtered_distance_m or 0.0,
                forward_obstacle_distance_m=self._filtered_distance_m,
                scale_confidence=0.3,
                is_valid_reading=meas.is_valid
            )

        # Scale estimation strategy 1: Central feature depth agreement
        # If we have triangulated points near the image center (optical axis),
        # their depth z_triang should match the ToF distance d_tof.
        scale_candidate: Optional[float] = None
        if central_feature_depths and len(central_feature_depths) >= 3:
            median_depth = float(np.median(central_feature_depths))
            if median_depth > 0.05 and self._filtered_distance_m is not None:
                # Ratio provides metric scale correction factor
                scale_factor = self._filtered_distance_m / median_depth
                if 0.2 < scale_factor < 5.0:
                    scale_candidate = self._current_scale * scale_factor
                    self._scale_confidence = min(0.9, self._scale_confidence + 0.1)

        # Scale estimation strategy 2: Forward distance delta
        # When moving predominantly forward along the Z axis (t_z dominates):
        # delta_d = d_prev - d_curr ~ delta_z_movement
        if scale_candidate is None and self._prev_distance_m is not None and self._filtered_distance_m is not None:
            t_flat = np.ravel(t_rel_unit)
            tz = abs(float(t_flat[2]))
            tx = abs(float(t_flat[0]))
            ty = abs(float(t_flat[1]))

            # Dominant forward motion
            if tz > 0.6 and tz > (tx + ty):
                delta_d = self._prev_distance_m - self._filtered_distance_m
                # Moving forward: delta_d > 0; Moving backward: delta_d < 0
                step_m = abs(delta_d)
                # Filter out physically impossible step velocities (e.g. max 0.5m per frame at ~15Hz)
                if 0.005 <= step_m <= 0.3:
                    scale_candidate = step_m
                    self._scale_confidence = 0.7

        # Smooth scale update if valid candidate was computed
        if scale_candidate is not None:
            self._current_scale = (0.25 * scale_candidate) + (0.75 * self._current_scale)
        else:
            self._scale_confidence = max(0.2, self._scale_confidence - 0.02)

        # Store for next iteration
        if meas.is_valid:
            self._prev_distance_m = self._filtered_distance_m

        return FusionState(
            estimated_metric_scale=self._current_scale,
            filtered_distance_m=self._filtered_distance_m or 0.0,
            forward_obstacle_distance_m=self._filtered_distance_m,
            scale_confidence=self._scale_confidence,
            is_valid_reading=meas.is_valid
        )

    def reset(self) -> None:
        """Reset internal filter state."""
        self._prev_distance_m = None
        self._filtered_distance_m = None
        self._current_scale = self.default_scale
        self._scale_confidence = 0.5
