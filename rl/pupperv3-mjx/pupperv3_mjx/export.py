import numpy as np
from jax import numpy as jp


def fold_in_normalization(A, b, mean, std):
    A_prime = A / std[:, np.newaxis]
    b_prime = (b - (A.T @ (mean / std)[:, np.newaxis]).T)[0]
    return A_prime, b_prime


def convert_params(
    params,
    activation: str,
    action_scale: float,
    kp: float,
    kd: float,
    default_pose: np.ndarray,
    joint_upper_limits: np.ndarray,
    joint_lower_limits: np.ndarray,
    use_imu: bool,
    observation_history: int,
    final_activation: str = "tanh",
):
    mean, std = params[0].mean, params[0].std
    params_dict = params[1]["params"]
    layers = []
    for i, (layer_name, layer_params) in enumerate(params_dict.items()):
        is_first_layer = i == 0
        is_final_layer = i == len(params_dict) - 1
        bias = layer_params["bias"]
        kernel = layer_params["kernel"]
        if is_first_layer:
            kernel, bias = fold_in_normalization(A=kernel, b=bias, mean=mean, std=std)
            input_size = kernel.shape[0]
        if is_final_layer:
            bias, _ = jp.split(bias, 2, axis=-1)
            kernel, _ = jp.split(kernel, 2, axis=-1)

        # Determine the output shape from the bias length
        output_shape = len(bias)

        # Convert kernel to a nested list
        kernel_list = kernel.tolist()

        # Determine the input shape from the kernel shape
        # TODO: Determine whether this is necessary
        # input_shape = len(kernel_list[0])

        # Create layer dictionary
        layer_dict = {
            "type": "dense",
            "activation": activation if not is_final_layer else final_activation,
            "shape": [None, output_shape],
            "weights": [kernel_list, bias.tolist()],
        }

        # Add layer dictionary to layers list
        layers.append(layer_dict)

    # Create policy diction with additional metadata
    # for correctly running the policy including
    # imu, observation_history, kp, kd, default_pose
    # upper/lower limits, in shape, and finallly layers
    final_dict = {
        "use_imu": use_imu,
        "control_orientation": True,
        "observation_history": observation_history,
        "action_scale": action_scale,
        "kp": kp,
        "kd": kd,
        "default_joint_pos": np.array(default_pose).tolist(),
        "joint_upper_limits": np.array(joint_upper_limits).tolist(),
        "joint_lower_limits": np.array(joint_lower_limits).tolist(),
        "in_shape": [None, input_size],
        "layers": layers,
    }

    return final_dict
