"""Núcleo del reconstructor de volumen. Python puro + numpy; no importa ROS.

Comparte con falconscan_velocity.core la geometría de montaje, la validación de rangos, el
perfil en malla y el PassTracker.
"""
from .checks import INVALIDATING, RESULT_TEXT, WARNINGS
from .integration import IntegrationResult, Profile, integrate
from .odometry import VelocityHistory, VelocitySample
from .params import VolumeParams
from .reconstruction import Reconstruction, build, surface_triangles, write_json, write_ply
from .reconstructor import CaptureStarted, CloudUpdate, Measurement, VolState, VolumeReconstructor
from .section import Section, SectionProcessor

__all__ = [
    "INVALIDATING", "RESULT_TEXT", "WARNINGS",
    "IntegrationResult", "Profile", "integrate",
    "VelocityHistory", "VelocitySample",
    "VolumeParams",
    "Reconstruction", "build", "surface_triangles", "write_json", "write_ply",
    "CaptureStarted", "CloudUpdate", "Measurement", "VolState", "VolumeReconstructor",
    "Section", "SectionProcessor",
]
