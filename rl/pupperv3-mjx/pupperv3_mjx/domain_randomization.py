import jax
from dataclasses import dataclass
from jax import numpy as jp
from typing import Tuple


def domain_randomize(
    sys,
    rng,
    friction_range: Tuple = (0.6, 1.4),
    kp_multiplier_range: Tuple = (0.75, 1.25),
    kd_multiplier_range: Tuple = (0.5, 2.0),
    body_com_x_shift_range: Tuple = (-0.03, 0.03),
    body_com_y_shift_range: Tuple = (-0.01, 0.01),
    body_com_z_shift_range: Tuple = (-0.02, 0.02),
    body_inertia_scale_range: Tuple = (0.7, 1.3),
    body_mass_scale_range: Tuple = (0.7, 1.3),
):
    """Randomizes the friction, actuator kp, & actuator kd

    TODO: What is the dimension of rng? The number of environments?
    """

    @jax.vmap
    def rand(rng):
        _, key = jax.random.split(rng, 2)
        # friction
        friction = jax.random.uniform(key, (1,), minval=friction_range[0], maxval=friction_range[1])
        friction = sys.geom_friction.at[:, 0].set(friction)
        # actuator
        _, key_kp, key_kd = jax.random.split(key, 3)
        kp = (
            jax.random.uniform(
                key_kp, (1,), minval=kp_multiplier_range[0], maxval=kp_multiplier_range[1]
            )
            * sys.actuator_gainprm[:, 0]
        )
        kd = jax.random.uniform(
            key_kd, (1,), minval=kd_multiplier_range[0], maxval=kd_multiplier_range[1]
        ) * (-sys.actuator_biasprm[:, 2])

        gain = sys.actuator_gainprm.at[:, 0].set(kp)
        bias = sys.actuator_biasprm.at[:, 1].set(-kp).at[:, 2].set(-kd)

        _, key_com = jax.random.split(key_kd)
        body_com_shift = jax.random.uniform(
            key_com,
            (3,),
            minval=jp.array(
                [body_com_x_shift_range[0], body_com_y_shift_range[0], body_com_z_shift_range[0]]
            ),
            maxval=jp.array(
                [body_com_x_shift_range[1], body_com_y_shift_range[1], body_com_z_shift_range[1]]
            ),
        )
        body_com = sys.body_ipos.at[1].set(sys.body_ipos[1] + body_com_shift)

        # TODO(nathankau) think if we want to scale inertia uniformly or not
        # TODO(nathankau) do we want to randomize all links inertia or just main body?
        _, key_inertia = jax.random.split(key_com)
        body_inertia_scale = jax.random.uniform(
            key_inertia,
            sys.body_inertia.shape,
            minval=body_inertia_scale_range[0],
            maxval=body_inertia_scale_range[1],
        )
        body_inertia = sys.body_inertia * body_inertia_scale

        _, key_mass = jax.random.split(key_inertia)
        body_mass_scale = jax.random.uniform(
            key_mass,
            sys.body_mass.shape,
            minval=body_mass_scale_range[0],
            maxval=body_mass_scale_range[1],
        )
        body_mass = sys.body_mass * body_mass_scale

        return friction, gain, bias, body_com, body_inertia, body_mass

    friction, gain, bias, body_com, body_inertia, body_mass = rand(rng)

    in_axes = jax.tree_map(lambda x: None, sys)
    in_axes = in_axes.tree_replace(
        {
            "geom_friction": 0,
            "actuator_gainprm": 0,
            "actuator_biasprm": 0,
            "body_ipos": 0,
            "body_inertia": 0,
            "body_mass": 0,
        }
    )

    sys = sys.tree_replace(
        {
            "geom_friction": friction,
            "actuator_gainprm": gain,
            "actuator_biasprm": bias,
            "body_ipos": body_com,
            "body_inertia": body_inertia,
            "body_mass": body_mass,
        }
    )

    return sys, in_axes


@dataclass
class StartPositionRandomization:
    x_min: float
    x_max: float
    y_min: float
    y_max: float
    z_min: float
    z_max: float


def random_z_rotation_quaternion(rng):
    """Generates a random quaternion with a random yaw angle."""
    yaw = jax.random.uniform(rng, (1,), minval=-jp.pi, maxval=jp.pi)
    cos_yaw = jp.cos(yaw / 2)
    sin_yaw = jp.sin(yaw / 2)
    return jp.concatenate((cos_yaw, jp.zeros(2), sin_yaw))


def randomize_qpos(qpos: jp.array, start_position_config: StartPositionRandomization, rng):
    """Return qpos with randomized position of first body. Do not use rng again!"""

    key_x, key_y, key_z, key_yaw = jax.random.split(rng, 4)
    qpos = qpos.at[:3].set(
        jax.random.uniform(
            key_z,
            shape=(3,),
            minval=jp.array(
                (
                    start_position_config.x_min,
                    start_position_config.y_min,
                    start_position_config.z_min,
                )
            ),
            maxval=jp.array(
                (
                    start_position_config.x_max,
                    start_position_config.y_max,
                    start_position_config.z_max,
                )
            ),
        )
    )
    random_yaw_quat = random_z_rotation_quaternion(key_yaw)
    qpos = qpos.at[3:7].set(random_yaw_quat)
    return qpos
