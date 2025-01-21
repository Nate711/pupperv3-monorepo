"""
pytest test/test_domain_randomization.py
"""

from pupperv3_mjx import domain_randomization
from jax import numpy as jp
import jax
from pathlib import Path

from jax import random
from brax.io import mjcf


def test_randomize_qpos():
    start_position_config = domain_randomization.StartPositionRandomization(
        x_min=-0.5, x_max=0.5, y_min=-0.5, y_max=0.5, z_min=-0.5, z_max=0.5
    )

    rng = jax.random.PRNGKey(0)

    qpos = domain_randomization.randomize_qpos(
        jp.array([0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10], dtype=jp.float32), start_position_config, rng
    )

    expected_qpos = jp.array([-0.184, 0.165, 0.278, 0.709, 0.0, 0.0, 0.705, 7.0, 8.0, 9.0, 10.0])

    assert jp.isclose(
        qpos, expected_qpos, atol=1e-3
    ).all(), f"Expected: {expected_qpos}, but got: {qpos}"


def test_domain_randomize():
    # Create a mock sys object with necessary attributes
    ORIGINAL_MODEL_PATH = Path("test/test_pupper_model.xml")

    sys = mjcf.load(ORIGINAL_MODEL_PATH)

    # Generate a random key
    rng = random.PRNGKey(0)

    original_kp = sys.actuator_gainprm[:, 0]
    original_kd = -sys.actuator_biasprm[:, 2]
    original_body_inertia = sys.body_inertia[1]
    original_body_mass = sys.body_mass[1]
    original_body_com = sys.body_ipos[1]

    rngs = jax.random.split(rng, 10)

    # Call the domain_randomize function
    sys, in_axes = domain_randomization.domain_randomize(
        sys,
        rngs,
        friction_range=(2.0, 10.0),
        kp_multiplier_range=(1.1, 1.25),
        kd_multiplier_range=(1.5, 2.0),
        body_com_x_shift_range=(0.02, 0.04),
        body_com_y_shift_range=(0.02, 0.04),
        body_com_z_shift_range=(0.02, 0.04),
        body_inertia_scale_range=(1.5, 2.0),
        body_mass_scale_range=(1.5, 2.0),
    )

    # Check if the output sys has the attributes updated correctly
    assert sys.geom_friction.shape == (10, 23, 3)
    assert sys.actuator_gainprm.shape == (10, 12, 10)
    assert sys.actuator_biasprm.shape == (10, 12, 10)

    # Test friction changed
    assert (sys.geom_friction[:, :, 0] >= 2.0).all() and (sys.geom_friction[:, :, 0] <= 10.0).all()

    # Test actuator gains changed
    assert (sys.actuator_gainprm[:, :, 0] >= 1.1 * original_kp).all() and (
        sys.actuator_gainprm[:, :, 0] <= 1.25 * original_kp
    ).all()
    assert (-sys.actuator_biasprm[:, :, 2] >= 1.5 * original_kd).all() and (
        -sys.actuator_biasprm[:, :, 2] <= 2.0 * original_kd
    ).all()

    # Test body inertia changed
    assert (sys.body_inertia[:, 1] >= 1.5 * original_body_inertia).all() and (
        sys.body_inertia[:, 1] <= 2.0 * original_body_inertia
    ).all()

    # Test body mass changed
    assert (sys.body_mass[:, 1] >= 1.5 * original_body_mass).all() and (
        sys.body_mass[:, 1] <= 2.0 * original_body_mass
    ).all()

    # Test body com changed
    assert (sys.body_ipos[:, 1] - original_body_com >= 0.02).all() and (
        sys.body_ipos[:, 1] - original_body_com <= 0.04
    ).all()


if __name__ == "__main__":
    test_domain_randomize()
