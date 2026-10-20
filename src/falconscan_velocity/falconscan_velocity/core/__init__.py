"""Núcleo del algoritmo de velocidad. Python puro + numpy; no importa ROS."""
from .correlation import CorrelationResult, find_shift, masked_ncc
from .estimator import Status, VelocityEstimate, VelocityEstimator, VelocityParams
from .geometry import MountGeometry, ProfileGrid, height_profile, object_extent, polar_to_xz
from .scan import Scan, median_filter, validate_ranges

__all__ = [
    "CorrelationResult", "find_shift", "masked_ncc",
    "Status", "VelocityEstimate", "VelocityEstimator", "VelocityParams",
    "MountGeometry", "ProfileGrid", "height_profile", "object_extent", "polar_to_xz",
    "Scan", "median_filter", "validate_ranges",
]
