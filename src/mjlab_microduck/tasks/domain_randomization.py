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

from mjlab.managers.manager_term_config import EventTermCfg
from mjlab.managers.scene_entity_config import SceneEntityCfg
from mjlab.tasks.velocity import mdp as velocity_mdp

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
    dr: DomainRandomizationCfg,
    *,
    play: bool = False,
) -> None:
    """Add standard domain randomization events to *cfg* based on *dr*.

    Args:
        cfg: The environment configuration to modify in-place.
        dr: Domain randomization configuration with toggles and ranges.
        play: If ``True``, shorten push intervals for better visibility.
    """
    if dr.enable_com:
        cfg.events["randomize_com"] = EventTermCfg(
            func=velocity_mdp.randomize_field,
            mode="reset",
            domain_randomization=True,
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_base",)),
                "operation": "add",
                "field": "body_ipos",
                "ranges": (-dr.com_range, dr.com_range),
            },
        )

    if dr.enable_kp or dr.enable_kd:
        kp = dr.kp_range if dr.enable_kp else (1.0, 1.0)
        kd = dr.kd_range if dr.enable_kd else (1.0, 1.0)
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

    if dr.enable_mass_inertia:
        cfg.events["randomize_mass_inertia"] = EventTermCfg(
            func=microduck_mdp.randomize_mass_and_inertia,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot", body_names=("trunk_base",)),
                "scale_range": dr.mass_inertia_range,
            },
        )

    if dr.enable_joint_friction:
        cfg.events["randomize_joint_friction"] = EventTermCfg(
            func=velocity_mdp.randomize_field,
            mode="reset",
            domain_randomization=True,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)),
                "operation": "scale",
                "field": "dof_frictionloss",
                "ranges": dr.joint_friction_range,
            },
        )

    if dr.enable_joint_damping:
        cfg.events["randomize_joint_damping"] = EventTermCfg(
            func=velocity_mdp.randomize_field,
            mode="reset",
            domain_randomization=True,
            params={
                "asset_cfg": SceneEntityCfg("robot", joint_names=(r".*",)),
                "operation": "scale",
                "field": "dof_damping",
                "ranges": dr.joint_damping_range,
            },
        )

    if dr.enable_velocity_pushes:
        interval = (0.5, 1.0) if play else dr.velocity_push_interval_s
        cfg.events["push_robot"] = EventTermCfg(
            func=velocity_mdp.push_by_setting_velocity,
            mode="interval",
            interval_range_s=interval,
            params={
                "velocity_range": {
                    "x": dr.velocity_push_range,
                    "y": dr.velocity_push_range,
                },
                "asset_cfg": SceneEntityCfg("robot"),
            },
        )

    if dr.enable_imu_orientation:
        cfg.events["randomize_imu_orientation"] = EventTermCfg(
            func=microduck_mdp.randomize_imu_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_angle_deg": dr.imu_orientation_angle,
            },
        )

    if dr.enable_base_orientation:
        cfg.events["randomize_base_orientation"] = EventTermCfg(
            func=microduck_mdp.randomize_base_orientation,
            mode="reset",
            params={
                "asset_cfg": SceneEntityCfg("robot"),
                "max_pitch_deg": dr.base_orientation_max_pitch_deg,
                "max_roll_deg": dr.base_orientation_max_roll_deg,
            },
        )
