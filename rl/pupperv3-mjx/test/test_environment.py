"""
pytest -s test/test_environment.py
"""

import pytest
from pupperv3_mjx import environment, domain_randomization, config, obstacles
import jax
from jax import numpy as jp
from pathlib import Path
import xml.etree.ElementTree as ET
import mediapy as media
import os
from brax.io import mjcf


@pytest.fixture(scope="module")
def setup_environment():
    ORIGINAL_MODEL_PATH = Path("test/test_pupper_model.xml")
    PATH = Path("test/model_with_obstacles.xml")

    # Read and modify the model XML
    xml_str = ORIGINAL_MODEL_PATH.read_text()
    tree = ET.ElementTree(ET.fromstring(xml_str))

    # Add obstacles
    N_OBSTACLES = 10
    OBSTACLE_X_RANGE = (-5, 5)
    OBSTACLE_Y_RANGE = (-5, 5)
    OBSTACLE_HEIGHT = 0.02
    OBSTACLE_LENGTH = 6.0

    tree = obstacles.add_boxes_to_model(
        tree,
        n_boxes=N_OBSTACLES,
        x_range=OBSTACLE_X_RANGE,
        y_range=OBSTACLE_Y_RANGE,
        height=OBSTACLE_HEIGHT,
        length=OBSTACLE_LENGTH,
    )
    with open(PATH, "w+") as file:
        tree.write(file, encoding="unicode")

    # Load the system and set up environment kwargs
    sys = mjcf.load(ORIGINAL_MODEL_PATH.as_posix())
    JOINT_UPPER_LIMITS = sys.jnt_range[1:, 1]
    JOINT_LOWER_LIMITS = sys.jnt_range[1:, 0]
    DEFAULT_POSE = jp.array(
        [0.26, 0.0, -0.52, -0.26, 0.0, 0.52, 0.26, 0.0, -0.52, -0.26, 0.0, 0.52]
    )

    env_kwargs = dict(
        path=PATH.as_posix(),
        action_scale=0.75,
        observation_history=2,
        joint_lower_limits=JOINT_LOWER_LIMITS,
        joint_upper_limits=JOINT_UPPER_LIMITS,
        dof_damping=0.25,
        position_control_kp=5.0,
        foot_site_names=[
            "leg_front_r_3_foot_site",
            "leg_front_l_3_foot_site",
            "leg_back_r_3_foot_site",
            "leg_back_l_3_foot_site",
        ],
        torso_name="base_link",
        upper_leg_body_names=[
            "leg_front_r_2",
            "leg_front_l_2",
            "leg_back_r_2",
            "leg_back_l_2",
        ],
        lower_leg_body_names=[
            "leg_front_r_3",
            "leg_front_l_3",
            "leg_back_r_3",
            "leg_back_l_3",
        ],
        resample_velocity_step=100,
        linear_velocity_x_range=[-0.75, 0.75],
        linear_velocity_y_range=[-0.5, 0.5],
        angular_velocity_range=[-2.0, 2.0],
        maximum_pitch_command=30,  # degrees
        maximum_roll_command=30,  # degrees
        default_pose=DEFAULT_POSE,
        start_position_config=domain_randomization.StartPositionRandomization(
            x_min=-1.0, x_max=1.0, y_min=-1.0, y_max=1.0, z_min=0.18, z_max=0.24
        ),
        reward_config=config.get_config(),
        kick_vel=1.0,
        kick_probability=0.04,
        terminal_body_z=0.1,
        early_termination_step_threshold=500,
    )

    return env_kwargs


def test_get_obs(setup_environment):
    env_kwargs = setup_environment
    env = environment.PupperV3Env(**env_kwargs)

    rng = jax.random.PRNGKey(0)
    state = env.reset(rng)
    obs_history = jp.zeros(env._observation_history * env.observation_dim, dtype=float)

    # Call _get_obs method
    obs = env._get_obs(state.pipeline_state, state.info, obs_history)

    # Check the shape of the observation
    assert obs.shape == (env._observation_history * env.observation_dim,)

    # Check that the observation values are within expected range
    assert jp.all(obs >= -100.0) and jp.all(obs <= 100.0)


def test_get_obs_imu_sampling(setup_environment):
    env_kwargs = setup_environment

    # The imu sample from 2 time steps ago will always be sampled
    env_kwargs["imu_latency_distribution"] = jp.array([0, 0, 1])
    env = environment.PupperV3Env(**env_kwargs)

    rng = jax.random.PRNGKey(0)
    state = env.reset(rng)
    obs_history = jp.zeros(env._observation_history * env.observation_dim, dtype=float)

    state.info["imu_buffer"] = jp.zeros((6, 3), dtype=float)
    # Set the 2nd oldest element to ones. The oldest element will pop out.
    expected_imu_data = jp.arange(6)
    state.info["imu_buffer"] = state.info["imu_buffer"].at[:, -2].set(expected_imu_data)

    # Call _get_obs method
    obs = env._get_obs(state.pipeline_state, state.info, obs_history)

    # Check that the imu buffer is being sampled correctly
    assert jp.allclose(obs[:6], expected_imu_data, atol=1e-5)


def test_pupper_environment_with_video(setup_environment):
    helper_test_pupper_environment(setup_environment, write_video=True)


# def test_pupper_environment_without_video(setup_environment):
#     helper_test_pupper_environment(setup_environment, write_video=False)


def helper_test_pupper_environment(setup_environment, write_video):
    env_kwargs = setup_environment

    # Reset environments since internals may be overwritten by tracers from the
    # domain randomization function.
    eval_env = environment.PupperV3Env(**env_kwargs)

    # produces a vectorized system so doesn't work with this code
    # rngs = jax.random.split(jax.random.PRNGKey(0), 2)
    # v_sys, _ = domain_randomization.domain_randomize(eval_env.sys, rngs)

    # Initialize the state
    rng = jax.random.PRNGKey(0)
    jit_reset = jax.jit(eval_env.reset)
    jit_step = jax.jit(eval_env.step)
    # jit_reset = eval_env.reset
    # jit_step = eval_env.step

    state = jit_reset(rng)
    state.info["command"] = jp.array([0.5, 0, 0])

    rollout = [state.pipeline_state]

    # Grab a trajectory
    n_steps = 200
    render_every = 2

    for i in range(n_steps):
        print("Step: ", i, end=" ")
        act_rng, rng = jax.random.split(rng)
        # ctrl = jp.array(0.5 * np.random.uniform(low=-1.0, high=1.0, size=eval_env.sys.nu))
        ctrl = jp.ones(eval_env.sys.nu)
        state = jit_step(state, ctrl)
        rollout.append(state.pipeline_state)
        print(
            "rng: ",
            state.info["rng"],
            "step: ",
            state.info["step"],
            "done: ",
            state.done,
            "command: ",
            state.info["command"],
            "body orientation: ",
            state.info["desired_world_z_in_body_frame"],
            "knee collision: ",
            state.info["rewards"]["knee_collision"],
            "body collision: ",
            state.info["rewards"]["body_collision"],
            "tracking orientation: ",
            state.info["rewards"]["tracking_orientation"],
        )

    if write_video:
        print("Writing video")
        media.write_video(
            "test_video.mp4",
            eval_env.render(rollout[::render_every], camera="tracking_cam"),
            fps=1.0 / eval_env.dt / render_every,
        )

        # Check if the video was created
        assert os.path.exists("test_video.mp4")


if __name__ == "__main__":
    pytest.main()
