"""Núcleo del algoritmo de velocidad. Python puro + numpy; no importa ROS."""
from .calibration import (FloorFit, MountCalibration, calibrate_mount, detect_direction, fit_floor,
                          fit_floor_offset, offset_from_down)
from .correlation import CorrelationResult, find_shift, masked_ncc
from .estimator import Status, VelocityEstimate, VelocityEstimator, VelocityParams
from .pass_tracker import ACTIVE_STATES, PassEvent, PassFlag, PassState, PassTracker
from .pipeline import VelocityPipeline
from .geometry import MountGeometry, ProfileGrid, fan_mask, height_profile, object_extent, polar_to_xz
from .scan import Scan, median_filter, validate_ranges

__all__ = [
    "FloorFit", "MountCalibration", "calibrate_mount", "detect_direction", "fit_floor", "fit_floor_offset",
    "offset_from_down",
    "CorrelationResult", "find_shift", "masked_ncc",
    "Status", "VelocityEstimate", "VelocityEstimator", "VelocityParams",
    "ACTIVE_STATES", "PassEvent", "PassFlag", "PassState", "PassTracker", "VelocityPipeline",
    "MountGeometry", "ProfileGrid", "fan_mask", "height_profile", "object_extent", "polar_to_xz",
    "Scan", "median_filter", "validate_ranges",
]
