from datetime import datetime
import matplotlib.pyplot as plt
import difflib
import re
import xml.etree.ElementTree as ET
from typing import List, Callable, Tuple
import mediapy as media
import os
import wandb
import jax
from jax import numpy as jp


def circular_buffer_push_back(buffer: jax.Array, new_value: jax.Array) -> jax.Array:
    """
    Shift a circular buffer back by one step and set the last element to a new value.
    The newest element will be at buf[:, -1]

    Args:
        buffer (jax.Array): The circular buffer. Dimensions: (buffer_size, buffer_shape).
        new_value (jax.Array): The new value to set at the last index. Dimensions: (buffer_shape).
    Returns:
        jax.Array: The updated circular buffer.
    """
    buffer = jp.roll(buffer, shift=-1, axis=1)
    return buffer.at[:, -1].set(new_value)


def circular_buffer_push_front(buffer: jax.Array, new_value: jax.Array) -> jax.Array:
    """
    Shift a circular buffer forward by one step and set the first element to a new value.
    The newest element will be at buf[:, 0]

    Args:
        buffer (jax.Array): The circular buffer. Dimensions: (buffer_size, buffer_shape).
        new_value (jax.Array): The new value to set at the first index. Dimensions: (buffer_shape).
    Returns:
        jax.Array: The updated circular buffer.
    """
    buffer = jp.roll(buffer, shift=1, axis=1)
    return buffer.at[:, 0].set(new_value)


def sample_lagged_value(
    rng: jax.Array, buffer_newest_first: jax.Array, new_value: jax.Array, distribution: jax.Array
) -> Tuple[jax.Array, jax.Array]:
    """
    Sample a value from a circular buffer with a lagged distribution.
    Args:
        rng (jax.Array): The random number generator key.
        buffer_newest_first (jax.Array): The circular buffer with the newest element up front.
        new_value (jax.Array): The new value to set at the first index.
        distribution (jax.Array): The distribution to sample the lagged value from.
    Returns:
        Tuple[jax.Array, jax.Array]: The sampled value and the updated circular buffer.
    """
    buffer_newest_first = circular_buffer_push_front(buffer_newest_first, new_value)
    return jax.random.choice(rng, buffer_newest_first, axis=1, p=distribution), buffer_newest_first


def progress(
    num_steps: int,
    metrics: dict,
    times: list,
    x_data: list,
    y_data: list,
    ydataerr: list,
    num_timesteps: int,
    min_y: float,
    max_y: float,
):
    """
    Update and display a progress plot with error bars.

    Args:
    num_steps (int): The current number of steps in the environment.
    metrics (dict): A dictionary containing evaluation metrics.
    times (list): A list to append the current time.
    x_data (list): A list to append the current number of steps.
    y_data (list): A list to append the current episode reward.
    ydataerr (list): A list to append the standard deviation of the episode reward.
    num_timesteps (int): The total number of timesteps for the x-axis limit.
    min_y (float): The minimum y-axis value.
    max_y (float): The maximum y-axis value.
    """
    times.append(datetime.now())
    x_data.append(num_steps)
    y_data.append(metrics["eval/episode_reward"])
    ydataerr.append(metrics["eval/episode_reward_std"])

    plt.xlim([0, num_timesteps * 1.25])
    plt.ylim([min_y, max_y])

    plt.xlabel("# environment steps")
    plt.ylabel("reward per episode")
    plt.title(f"y={y_data[-1]:.3f}")

    plt.errorbar(x_data, y_data, yerr=ydataerr)
    plt.show()

    wandb.log(metrics, step=num_steps)


def fuzzy_search(obj, search_str: str, cutoff: float = 0.6):
    """
    Perform a fuzzy search on the properties of an object.

    Args:
    obj: The object to search through.
    search_str (str): The string to match properties against.
    cutoff (float): The cutoff for matching ratio (0.0 to 1.0), higher means more strict matching.

    Returns:
    List[Tuple[str, float]]: A list of tuples containing (property_name, match_ratio) that match
    the search string.
    """
    results = []

    # Get all properties of the object
    properties = dir(obj)

    # Search for fuzzy matches
    for prop in properties:
        ratio = difflib.SequenceMatcher(None, search_str, prop).ratio()
        if ratio >= cutoff:
            results.append((prop, ratio))

    # Sort results by match ratio in descending order
    results.sort(key=lambda x: x[1], reverse=True)

    return results


def set_mjx_custom_options(tree: ET.ElementTree, max_contact_points: int, max_geom_pairs: int):
    """
    Set custom options for a MuJoCo XML model.

    Args:
    tree (ET.ElementTree): The XML tree of the MuJoCo model.
    max_contact_points (int): The maximum number of contact points.
    max_geom_pairs (int): The maximum number of geometry pairs.

    Returns:
    ET.ElementTree: The updated XML tree.
    """
    root = tree.getroot()
    custom = root.find("custom")
    if custom is not None:
        for numeric in custom.findall("numeric"):
            name = numeric.get("name")
            if name == "max_contact_points":
                numeric.set("data", str(max_contact_points))
            elif name == "max_geom_pairs":
                numeric.set("data", str(max_geom_pairs))

        return tree
    return None


def set_robot_starting_position(
    tree: ET.ElementTree, starting_pos: List, starting_quat: List = None
):
    """
    Change the starting position of the robot in the XML MuJoCo model file.

    Args:
    tree (ET.ElementTree): The XML tree of the MuJoCo model.
    starting_pos (List[float]): The starting position [x, y, z].
    starting_quat (List[float], optional): The starting quaternion [x, y, z, w].

    Returns:
    ET.ElementTree: The updated XML tree.
    """

    body = tree.find(".//worldbody/body[@name='base_link']")
    body.set("pos", f"{starting_pos[0]} {starting_pos[1]} {starting_pos[2]}")
    if starting_quat is not None:
        body.set(
            "quat", f"{starting_quat[0]} {starting_quat[1]} {starting_quat[2]} {starting_quat[3]}"
        )

    home_position = tree.find(".//keyframe/key[@name='home']")
    qpos_scalar = list(map(float, re.split(r"\s+", home_position.get("qpos").strip())))
    qpos_scalar[:3] = starting_pos
    if starting_quat is not None:
        qpos_scalar[3:7] = starting_quat
    updated_qpos = " ".join(map(str, qpos_scalar))
    home_position.set("qpos", updated_qpos)
    return tree


def visualize_policy(
    current_step,
    make_policy_fn,
    params,
    eval_env,
    jit_step: Callable,
    jit_reset: Callable,
    output_folder: str,
    vx: float = 0.5,
    vy: float = 0.4,
    wz: float = 1.5,
):
    """
    Visualize a policy by creating a video of the robot's behavior.

    Args:
    current_step (int): The current training step.
    make_policy_fn (Callable): A function to create the policy.
    params (Tuple): The parameters for the policy.
    eval_env: The evaluation environment.
    jit_step (Callable): A JIT-compiled function to perform a step in the environment.
    jit_reset (Callable): A JIT-compiled function to reset the environment.
    output_folder (str): The folder to save the output video.
    vx (float): The forward/backward velocity.
    vy (float): The left/right velocity.
    wz (float): The rotational velocity.
    """

    inference_fn = make_policy_fn((params[0], params[1].policy))
    jit_inference_fn = jax.jit(inference_fn)

    # Make robot go forward, back, left, right
    command_seq = jp.array(
        [
            [0.0, 0.0, 0.0],
            [vx, 0.0, 0.0],
            [-vx, 0.0, 0.0],
            [0.0, vy, 0.0],
            [0.0, -vy, 0.0],
            [0.0, 0.0, wz],
            [0.0, 0.0, -wz],
        ]
    )

    # initialize the state
    rng = jax.random.PRNGKey(0)
    state = jit_reset(rng)
    state.info["command"] = command_seq[0]
    rollout = [state.pipeline_state]

    # grab a trajectory
    n_steps = 560
    render_every = 2
    ctrls = []

    for i in range(n_steps):
        act_rng, rng = jax.random.split(rng)

        # Change command every 80 steps
        state.info["command"] = command_seq[int(i / 80)]

        ctrl, _ = jit_inference_fn(state.obs, act_rng)
        state = jit_step(state, ctrl)
        rollout.append(state.pipeline_state)
        ctrls.append(ctrl)

    filename = os.path.join(output_folder, f"step_{current_step}_policy.mp4")
    fps = int(1.0 / eval_env.dt / render_every)
    media.write_video(
        filename,
        eval_env.render(rollout[::render_every], camera="tracking_cam"),
        fps=fps,
    )
    wandb.log(
        {
            "eval/video/command/vx": vx,
            "eval/video/command/vy": vy,
            "eval/video/command/wz": wz,
            "eval/video": wandb.Video(filename, format="mp4"),
        },
        step=current_step,
    )


def activation_fn_map(activation_name: str):
    """
    Map an activation function name to its corresponding JAX function.

    Args:
    activation_name (str): The name of the activation function (e.g., 'relu', 'sigmoid').

    Returns:
    Callable: The corresponding JAX activation function.
    """
    activation_name = activation_name.lower()
    return {
        "relu": jax.nn.relu,
        "sigmoid": jax.nn.sigmoid,
        "elu": jax.nn.elu,
        "tanh": jp.tanh,
        "softmax": jax.nn.softmax,
    }[activation_name]
