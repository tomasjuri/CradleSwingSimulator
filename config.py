"""
Configuration for the Kolebka rocking cradle simulation.
All physical parameters are in SI units (kg, m, s) unless noted.
"""

from dataclasses import dataclass, field
import numpy as np


@dataclass
class CradleConfig:
    """Physical parameters of the cradle body."""
    mass: float = 5.0              # kg – total mass of the empty cradle
    body_width: float = 0.40       # m  – half-width of the egg-shaped body
    body_height: float = 0.25      # m  – half-height of the egg-shaped body
    cm_height: float = 0.15        # m  – centre-of-mass height above rocker origin


@dataclass
class RockerConfig:
    """
    Rocker (leg) profile parameters.
    The curve in the body frame is  y(x) = R * |x/W|^p
    """
    R: float = 0.08                # m  – depth of the rocker curve
    W: float = 0.20                # m  – half-width of the rocker contact zone
    p: float = 2.0                 # shape exponent: 2.0 = circular (A), 1.0 = V-shape (B)


@dataclass
class WeightConfig:
    """Moving weight (závaží) parameters."""
    mass: float = 1.0              # kg
    rail_height: float = 0.15      # m  – height of the rail above rocker origin (in body frame)
    rail_half_span: float = 0.15   # m  – max displacement of the weight from centre


@dataclass
class MotorConfig:
    """Stepper motor constraints."""
    max_acceleration: float = 2.0  # m/s²
    max_velocity: float = 0.3      # m/s


@dataclass
class SurfaceConfig:
    """Ground surface / damping model."""
    damping: float = 0.15          # N·m·s/rad – rolling resistance coefficient
    # Higher for carpet (~0.5), lower for hard floor (~0.05)


@dataclass
class ControlConfig:
    """Controller tuning."""
    target_amplitude: float = 0.15  # rad – desired steady-state tilt amplitude
    gain: float = 1.0               # controller gain multiplier (0..1)
    enabled: bool = True


@dataclass
class SimConfig:
    """Simulation meta-parameters."""
    dt: float = 0.0005             # s – physics timestep (small for stability)
    speed: float = 1.0             # simulation speed multiplier
    gravity: float = 9.81          # m/s²
    max_theta: float = 0.6         # rad – safety clamp for tilt angle


@dataclass
class DisplayConfig:
    """Rendering / window settings."""
    window_width: int = 1400
    window_height: int = 800
    fps: int = 60
    sim_view_width: int = 800      # pixels for the left cradle view
    pixels_per_meter: float = 600.0  # zoom factor


@dataclass
class Config:
    """Top-level config aggregating all sub-configs."""
    cradle: CradleConfig = field(default_factory=CradleConfig)
    rocker: RockerConfig = field(default_factory=RockerConfig)
    weight: WeightConfig = field(default_factory=WeightConfig)
    motor: MotorConfig = field(default_factory=MotorConfig)
    surface: SurfaceConfig = field(default_factory=SurfaceConfig)
    control: ControlConfig = field(default_factory=ControlConfig)
    sim: SimConfig = field(default_factory=SimConfig)
    display: DisplayConfig = field(default_factory=DisplayConfig)
