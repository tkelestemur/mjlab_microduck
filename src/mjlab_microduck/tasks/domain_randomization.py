"""Shared domain randomization configuration for all Microduck tasks.

Centralises the DR toggle flags, default ranges, and event-building logic
that was previously duplicated across the velocity, standup, ground-pick,
and imitation environment configs.

Per-task overrides are passed as keyword arguments to
``add_domain_randomization_events()``.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import TYPE_CHECKING

from mjlab.envs.mdp import dr
from mjlab.envs.mdp.events import push_by_setting_velocity
from mjlab.managers import EventTermCfg, SceneEntityCfg

from mjlab_microduck.tasks import mdp as microduck_mdp

if TYPE_CHECKING:
    from mjlab.envs import ManagerBasedRlEnvCfg


# ---------------------------------------------------------------------------
# Default ranges (shared across tasks unless overridden)
# ---------------------------------------------------------------------------

# Conservative ranges proven to be stable
DEFAULT_COM_RANGE = 0.003  # ±3 mm
DEFAULT_MASS_INERTIA_RANGE = (0.95, 1.05)  # ±5 %
DEFAULT_KP_RANGE = (0.85, 1.15)  # ±15 %
DEFAULT_KD_RANGE = (0.9, 1.1)  # ±10 %
DEFAULT_JOINT_FRICTION_RANGE = (0.98, 1.02)  # ±2 % (very conservative)
DEFAULT_JOINT_DAMPING_RANGE = (0.98, 1.02)  # ±2 % (very conservative)
DEFAULT_VELOCITY_PUSH_INTERVAL_S = (3.0, 6.0)
DEFAULT_VELOCITY_PUSH_RANGE = (-0.3, 0.3)  # m/s
DEFAULT_IMU_ORIENTATION_ANGLE = 1.0  # degrees
DEFAULT_BASE_ORIENTATION_MAX_PITCH_DEG = 10.0
DEFAULT_BASE_ORIENTATION_MAX_ROLL_DEG = 5.0


@dataclass
class DomainRandomizationCfg:
    """All DR toggles and ranges for a single task, with sensible defaults."""

    # Toggles
    enable_com: bool = True
    enable_kp: bool = True
    enable_kd: bool = True
    enable_mass_inertia: bool = True
    enable_joint_friction: bool = False
    enable_joint_damping: bool = False
    enable_velocity_pushes: bool = True
    enable_imu_orientation: bool = True
    enable_base_orientation: bool = False

    # Ranges
    com_range: float = DEFAULT_COM_RANGE
    mass_inertia_range: tuple[float, float] = DEFAULT_MASS_INERTIA_RANGE
    kp_range: tuple[float, float] = DEFAULT_KP_RANGE
    kd_range: tuple[float, float] = DEFAULT_KD_RANGE
    joint_friction_range: tuple[float, float] = DEFAULT_JOINT_FRICTION_RANGE
    joint_damping_range: tuple[float, float] = DEFAULT_JOINT_DAMPING_RANGE
    velocity_push_interval_s: tuple[float, float] = DEFAULT_VELOCITY_PUSH_INTERVAL_S
    velocity_push_range: tuple[float, float] = DEFAULT_VELOCITY_PUSH_RANGE
    imu_orientation_angle: float = DEFAULT_IMU_ORIENTATION_ANGLE
    base_orientation_max_pitch_deg: float = DEFAULT_BASE_ORIENTATION_MAX_PITCH_DEG
    base_orientation_max_roll_deg: float = DEFAULT_BASE_ORIENTATION_MAX_ROLL_DEG


def add_domain_randomization_events(
    cfg: ManagerBasedRlEnvCfg,
    dr_cfg: DomainRandomizationCfg,
    *,
    play: bool = False,
) -> None:
    """Add standard domain randomization events to *cfg* based on *dr_cfg*.

    Args:
        cfg: The environment configuration to modify in-place.
        dr_cfg: Domain randomization configuration with toggles and ranges.
        play: If ``True``, shorten push intervals for better visibility.
    """
    if dr_cfg.enable_com:
        cfg.events["randomize_com"] = EventTermCfg(
            func=dr.body_ipos,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_base",)),
                "operation": "add",
                "ranges": (-dr_cfg.com_range, dr_cfg.com_range),
            },
        )

    if dr_cfg.enable_kp or dr_cfg.enable_kd:
        kp = dr_cfg.kp_range if dr_cfg.enable_kp else (1.0, 1.0)
        kd = dr_cfg.kd_range if dr_cfg.enable_kd else (1.0, 1.0)
        cfg.events["randomize_motor_gains"] = EventTermCfg(
            func=microduck_mdp.randomize_delayed_actuator_gains,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "operation": "scale",
                "kp_range": kp,
                "kd_range": kd,
            },
        )

    if dr_cfg.enable_mass_inertia:
        cfg.events["randomize_mass_inertia"] = EventTermCfg(
            func=microduck_mdp.randomize_mass_and_inertia,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_base",)),
                "scale_range": dr_cfg.mass_inertia_range,
            },
        )

    if dr_cfg.enable_joint_friction:
        cfg.events["randomize_joint_friction"] = EventTermCfg(
            func=dr.joint_friction,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)),
                "operation": "scale",
                "ranges": dr_cfg.joint_friction_range,
            },
        )

    if dr_cfg.enable_joint_damping:
        cfg.events["randomize_joint_damping"] = EventTermCfg(
            func=dr.joint_damping,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)),
                "operation": "scale",
                "ranges": dr_cfg.joint_damping_range,
            },
        )

    if dr_cfg.enable_velocity_pushes:
        interval = (0.5, 1.0) if play else dr_cfg.velocity_push_interval_s
        cfg.events["push_robot"] = EventTermCfg(
            func=push_by_setting_velocity,
            mode="interval",
            interval_range_s=interval,
            params={
                "velocity_range": {
                    "x": dr_cfg.velocity_push_range,
                    "y": dr_cfg.velocity_push_range,
                },
                "asset_cfg": SceneEntityCfg("robot"),
            },
        )

    if dr_cfg.enable_imu_orientation:
        cfg.events["randomize_imu_orientation"] = EventTermCfg(
            func=microduck_mdp.randomize_imu_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_angle_deg": dr_cfg.imu_orientation_angle,
            },
        )

    if dr_cfg.enable_base_orientation:
        cfg.events["randomize_base_orientation"] = EventTermCfg(
            func=microduck_mdp.randomize_base_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_pitch_deg": dr_cfg.base_orientation_max_pitch_deg,
                "max_roll_deg": dr_cfg.base_orientation_max_roll_deg,
            },
        )
