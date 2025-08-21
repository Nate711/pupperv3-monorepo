#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pupper MJX RL Training Script

Modularized version of the original Colab notebook for local execution.
Authors: Google (thanks Baruch!) + Nathan + Gabrael
"""

import os
import sys
import logging
import subprocess
import time
import itertools
import functools
import json
from datetime import datetime
from pathlib import Path
from typing import Callable, NamedTuple, Optional, Union, List, Any, Dict, Sequence, Tuple

import hydra
from omegaconf import DictConfig, OmegaConf
import numpy as np
import jax
from jax import numpy as jp
import matplotlib.pyplot as plt
import plotly.express as px
import plotly.graph_objects as go

# ML/RL imports
from brax import base, envs, math
from brax.base import Base, Motion, Transform
from brax.envs.base import Env, PipelineEnv, State
from brax.mjx.base import State as MjxState
from brax.training.agents.ppo import train as ppo
from brax.training.agents.ppo import networks as ppo_networks
from brax.io import html, mjcf, model
from etils import epath
from flax import struct
from ml_collections import config_dict
import mujoco
from mujoco import mjx

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Configure numpy printing
np.set_printoptions(precision=3, suppress=True, linewidth=100)

# Set XLA flags for GPU optimization
xla_flags = os.environ.get("XLA_FLAGS", "")
xla_flags += " --xla_gpu_triton_gemm_any=True"
os.environ["XLA_FLAGS"] = xla_flags

# Optional imports
try:
    import mediapy as media
except ImportError:
    logger.warning("mediapy not available. Video rendering will be disabled.")
    media = None

try:
    import wandb
except ImportError:
    logger.warning("wandb not available. Experiment tracking will be disabled.")
    wandb = None


class SystemChecker:
    """Handles system validation and setup."""

    @staticmethod
    def check_gpu() -> None:
        """Check GPU availability."""
        try:
            result = subprocess.run(["nvidia-smi"], capture_output=True, text=True)
            if result.returncode == 0:
                logger.info("GPU detected.")
            else:
                logger.warning("GPU not detected. Training will be slower on CPU.")
        except FileNotFoundError:
            logger.warning("nvidia-smi not found. GPU status unknown.")

    @staticmethod
    def verify_mujoco() -> None:
        """Verify MuJoCo installation."""
        try:
            mujoco.MjModel.from_xml_string("<mujoco/>")
            logger.info("MuJoCo installation verified.")
        except Exception as e:
            raise RuntimeError(f"MuJoCo installation check failed: {e}")

    @staticmethod
    def setup_environment() -> None:
        """Setup environment variables."""
        os.environ["MUJOCO_GL"] = "egl"
        logger.info("Environment variables configured.")


class ModelManager:
    """Handles model setup and modifications."""

    def __init__(self, config: DictConfig):
        self.config = config
        self.work_dir = Path.cwd()

    def setup_repositories(self) -> None:
        """Clone and setup required repositories."""
        # Setup pupperv3-mjx repo
        pupperv3_dir = self.work_dir / "pupperv3_mjx"
        if not pupperv3_dir.exists():
            logger.info("Cloning pupperv3-mjx repository...")
            subprocess.run(
                ["git", "clone", "-b", self.config.pupperv3_mjx.branch, "https://github.com/Nate711/pupperv3-mjx.git"],
                check=True,
            )
            subprocess.run(["mv", "pupperv3-mjx", "pupperv3_mjx"], check=True)

        # Install pupperv3-mjx
        subprocess.run([sys.executable, "-m", "pip", "install", "-e", str(pupperv3_dir)], check=True)

        # Setup pupper model repo
        model_dir = self.work_dir / "pupper_v3_description"
        if not model_dir.exists():
            logger.info("Cloning pupper_v3_description repository...")
            subprocess.run(
                ["git", "clone", self.config.simulation.model_repo, "-b", self.config.simulation.model_branch],
                check=True,
            )

    def modify_model(self) -> str:
        """Modify robot model and return path to modified model."""
        import xml.etree.ElementTree as ET
        from pupperv3_mjx import utils, obstacles

        # Load original model
        xml_str = epath.Path(self.config.simulation.original_model_path).read_text()
        tree = ET.ElementTree(ET.fromstring(xml_str))

        # Set MJX custom options
        tree = utils.set_mjx_custom_options(
            tree,
            max_contact_points=self.config.simulation.max_contact_points,
            max_geom_pairs=self.config.simulation.max_geom_pairs,
        )

        # Add obstacles
        tree = obstacles.add_boxes_to_model(
            tree,
            n_boxes=self.config.training.n_obstacles,
            x_range=self.config.training.obstacle_x_range,
            y_range=self.config.training.obstacle_y_range,
            height=self.config.training.obstacle_height,
            length=self.config.training.obstacle_length,
        )

        # Add height field if configured
        if self.config.training.height_field_random or self.config.training.height_field_steps:
            self._add_height_field(tree)

        # Save modified model
        model_path = self.config.simulation.model_path
        with open(model_path, "w+") as file:
            tree.write(file, encoding="unicode")

        logger.info(f"Modified model saved to: {model_path}")
        return model_path

    def _add_height_field(self, tree):
        """Add height field to the model."""
        if self.config.training.height_field_random:
            # Generate random height field
            noise = np.array(
                jax.random.uniform(
                    jax.random.PRNGKey(0),
                    (self.config.training.height_field_grid_size, self.config.training.height_field_grid_size),
                )
            )

            area_noise = jax.random.uniform(
                jax.random.PRNGKey(1),
                (
                    int(self.config.training.height_field_grid_size // self.config.training.height_field_radius_x),
                    int(self.config.training.height_field_grid_size // self.config.training.height_field_radius_y),
                ),
            )
            upscaled_area_noise = np.array(
                jax.image.resize(
                    image=area_noise,
                    shape=(self.config.training.height_field_grid_size, self.config.training.height_field_grid_size),
                    method="nearest",
                )
            )
            scaled_noise = noise * upscaled_area_noise

        elif self.config.training.height_field_steps:
            # Generate step height field
            steps = jax.random.uniform(
                jax.random.PRNGKey(0),
                (
                    self.config.training.height_field_grid_size // self.config.training.height_field_step_size,
                    self.config.training.height_field_grid_size // self.config.training.height_field_step_size,
                ),
            )
            scaled_noise = np.array(
                jax.image.resize(
                    image=steps,
                    shape=(self.config.training.height_field_grid_size, self.config.training.height_field_grid_size),
                    method="nearest",
                )
            )

        # Add height field to XML
        import xml.etree.ElementTree as ET

        root = tree.getroot()
        worldbody = root.find("worldbody")
        asset = root.find("asset")

        ET.SubElement(
            asset,
            "hfield",
            name="hfield_geom",
            nrow=f"{self.config.training.height_field_grid_size}",
            ncol=f"{self.config.training.height_field_grid_size}",
            elevation=" ".join(scaled_noise.astype(str).flatten().tolist()),
            size=f"{self.config.training.height_field_radius_x} {self.config.training.height_field_radius_y} {self.config.training.height_field_elevation_z} {self.config.training.height_field_base_z}",
        )

        ET.SubElement(
            worldbody,
            "geom",
            name="hfield_floor",
            type="hfield",
            hfield="hfield_geom",
            material="grid",
            conaffinity="1",
            contype="1",
            condim="3",
            group=self.config.training.height_field_group,
        )


class Trainer:
    """Handles the training process."""

    def __init__(self, config: DictConfig, model_path: str):
        self.config = config
        self.model_path = model_path
        self.output_folder = None
        self.wandb_run = None

    def setup_wandb(self) -> None:
        """Initialize Weights & Biases tracking."""
        if wandb is None:
            logger.warning("Wandb not available. Skipping experiment tracking.")
            return

        entity = "hands-on-robotics"
        try:
            wandb.init(
                entity=entity,
                project="pupperv3-mjx-rl",
                config=OmegaConf.to_container(self.config, resolve=True),
                save_code=True,
                settings={"_service_wait": 20, "init_timeout": 20},
            )
            logger.info("Using hands-on-robotics W&B team.")
        except Exception as e:
            logger.warning(f"hands-on-robotics team not found: {e}. Using personal project.")
            wandb.init(
                entity=None,
                project="pupperv3-mjx-rl",
                config=OmegaConf.to_container(self.config, resolve=True),
                save_code=True,
                settings={"_service_wait": 90, "init_timeout": 90},
            )

        self.wandb_run = wandb.run
        self.output_folder = f"output_{wandb.run.name}"
        os.makedirs(self.output_folder, exist_ok=True)

    def benchmark_model(self) -> float:
        """Benchmark the physics simulation."""
        from mujoco.mjx import benchmark

        jax.config.update("jax_default_matmul_precision", "bfloat16")
        sys = mjcf.load(self.model_path)
        jit_time, run_time, steps = benchmark(sys.mj_model, batch_size=self.config.training.ppo.num_envs)
        physics_steps_per_sec = steps / run_time
        logger.info(f"Physics steps per second: {physics_steps_per_sec:,.0f}")

        if self.wandb_run:
            try:
                wandb.run.summary["benchmark_physics_steps_per_sec"] = physics_steps_per_sec
            except:
                pass

        return physics_steps_per_sec

    def create_environment(self) -> Env:
        """Create and register the training environment."""
        from pupperv3_mjx import environment

        envs.register_environment("pupper", environment.PupperV3Env)

        # Get joint limits
        sys_temp = mjcf.load(self.config.simulation.original_model_path)
        joint_upper_limits = sys_temp.jnt_range[1:, 1]
        joint_lower_limits = sys_temp.jnt_range[1:, 0]

        env_kwargs = dict(
            path=self.model_path,
            action_scale=self.config.policy.action_scale,
            observation_history=self.config.policy.observation_history,
            joint_lower_limits=joint_lower_limits,
            joint_upper_limits=joint_upper_limits,
            dof_damping=self.config.training.dof_damping,
            position_control_kp=self.config.training.position_control_kp,
            foot_site_names=self.config.simulation.foot_site_names,
            torso_name=self.config.simulation.torso_name,
            upper_leg_body_names=self.config.simulation.upper_leg_body_names,
            lower_leg_body_names=self.config.simulation.lower_leg_body_names,
            resample_velocity_step=self.config.training.resample_velocity_step,
            linear_velocity_x_range=self.config.training.lin_vel_x_range,
            linear_velocity_y_range=self.config.training.lin_vel_y_range,
            angular_velocity_range=self.config.training.ang_vel_yaw_range,
            zero_command_probability=self.config.training.zero_command_probability,
            stand_still_command_threshold=self.config.training.stand_still_command_threshold,
            maximum_pitch_command=self.config.training.maximum_pitch_command,
            maximum_roll_command=self.config.training.maximum_roll_command,
            start_position_config=self.config.training.start_position_config,
            default_pose=jp.array(self.config.training.default_pose),
            desired_abduction_angles=jp.array(self.config.training.desired_abduction_angles),
            reward_config=self.config.reward,
            angular_velocity_noise=self.config.training.angular_velocity_noise,
            gravity_noise=self.config.training.gravity_noise,
            motor_angle_noise=self.config.training.motor_angle_noise,
            last_action_noise=self.config.training.last_action_noise,
            kick_vel=self.config.training.kick_vel,
            kick_probability=self.config.training.kick_probability,
            terminal_body_z=self.config.training.terminal_body_z,
            early_termination_step_threshold=self.config.training.early_termination_step_threshold,
            terminal_body_angle=self.config.training.terminal_body_angle,
            foot_radius=self.config.simulation.foot_radius,
            environment_timestep=self.config.training.environment_dt,
            physics_timestep=self.config.simulation.physics_dt,
            latency_distribution=jp.array(self.config.training.latency_distribution),
            imu_latency_distribution=jp.array(self.config.training.imu_latency_distribution),
            desired_world_z_in_body_frame=jp.array(self.config.training.desired_world_z_in_body_frame),
            use_imu=self.config.policy.use_imu,
        )

        return envs.get_environment("pupper", **env_kwargs)

    def train(self) -> Tuple[Any, Any]:
        """Execute the training process."""
        from pupperv3_mjx import domain_randomization, utils

        # Create environments
        env = self.create_environment()
        eval_env = self.create_environment()

        # Create visualization functions
        jit_reset = jax.jit(eval_env.reset)
        jit_step = jax.jit(eval_env.step)

        # Setup training function
        make_networks_factory = functools.partial(
            ppo_networks.make_ppo_networks,
            policy_hidden_layer_sizes=self.config.policy.hidden_layer_sizes,
            activation=utils.activation_fn_map(self.config.policy.activation),
        )

        train_fn = functools.partial(
            ppo.train,
            **(OmegaConf.to_container(self.config.training.ppo, resolve=True)),
            network_factory=make_networks_factory,
            randomization_fn=functools.partial(
                domain_randomization.domain_randomize,
                friction_range=self.config.training.friction_range,
                kp_multiplier_range=self.config.training.position_control_kp_multiplier_range,
                kd_multiplier_range=self.config.training.position_control_kd_multiplier_range,
                body_com_x_shift_range=self.config.training.body_com_x_shift_range,
                body_com_y_shift_range=self.config.training.body_com_y_shift_range,
                body_com_z_shift_range=self.config.training.body_com_z_shift_range,
                body_mass_scale_range=self.config.training.body_mass_scale_range,
                body_inertia_scale_range=self.config.training.body_inertia_scale_range,
            ),
            seed=28,
        )

        # Training tracking variables
        x_data = []
        y_data = []
        ydataerr = []
        times = [datetime.now()]

        def policy_params_fn(current_step, make_policy, params):
            utils.visualize_policy(
                current_step=current_step,
                make_policy=make_policy,
                params=params,
                eval_env=eval_env,
                jit_step=jit_step,
                jit_reset=jit_reset,
                output_folder=self.output_folder,
            )
            utils.save_checkpoint(
                current_step=current_step, make_policy=make_policy, params=params, checkpoint_path=self.output_folder
            )

        # Handle checkpoint loading
        checkpoint_kwargs = {}
        if self.config.training.checkpoint_run_number is not None:
            utils.download_checkpoint(
                entity_name="hands-on-robotics",
                project_name="pupperv3-mjx-rl",
                run_number=self.config.training.checkpoint_run_number,
                save_path="checkpoint",
            )
            checkpoint_kwargs["restore_checkpoint_path"] = Path("checkpoint").resolve()

        # Execute training
        logger.info("Starting training...")
        make_inference_fn, params, _ = train_fn(
            environment=env,
            progress_fn=functools.partial(
                utils.progress,
                times=times,
                x_data=x_data,
                y_data=y_data,
                ydataerr=ydataerr,
                num_timesteps=self.config.training.ppo.num_timesteps,
                min_y=0,
                max_y=40,
            ),
            eval_env=eval_env,
            policy_params_fn=policy_params_fn,
            **checkpoint_kwargs,
        )

        # Log training times
        logger.info(f"Time to JIT: {times[1] - times[0]}")
        logger.info(f"Time to train: {times[-1] - times[1]}")

        if self.wandb_run:
            wandb.run.summary["time_to_jit"] = (times[1] - times[0]).total_seconds()
            wandb.run.summary["time_to_train"] = (times[-1] - times[1]).total_seconds()

        return make_inference_fn, params

    def export_policy(self, params: Any) -> str:
        """Export policy for deployment."""
        from pupperv3_mjx import export

        params_rtneural = export.convert_params(
            jax.block_until_ready(params),
            activation=self.config.policy.activation,
            action_scale=self.config.policy.action_scale,
            kp=self.config.training.position_control_kp,
            kd=self.config.training.dof_damping,
            default_pose=self.config.training.default_pose,
            joint_upper_limits=self.config.simulation.joint_upper_limits,
            joint_lower_limits=self.config.simulation.joint_lower_limits,
            use_imu=self.config.policy.use_imu,
            observation_history=self.config.policy.observation_history,
            maximum_pitch_command=self.config.training.maximum_pitch_command,
            maximum_roll_command=self.config.training.maximum_roll_command,
            final_activation="tanh",
        )

        train_datetime = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        if self.wandb_run:
            name = f"policy_{wandb.run.name}_max_reward_{100:0.2f}.json"  # Placeholder reward
        else:
            name = f"policy_{train_datetime}_max_reward_{100:0.2f}.json"

        saved_policy_filename = os.path.join(self.output_folder, name)
        with open(saved_policy_filename, "w") as f:
            json.dump(params_rtneural, f)

        # Save model parameters
        model_path = os.path.join(self.output_folder, f"mjx_params_{train_datetime}")
        model.save_params(model_path, params)

        if self.wandb_run:
            wandb.log_model(path=saved_policy_filename, name=name)
            wandb.log_model(path=model_path, name=f"mjx_policy_network_{wandb.run.name}.pt")

        logger.info(f"Policy exported to: {saved_policy_filename}")
        return saved_policy_filename


@hydra.main(version_base=None, config_path="conf", config_name="config")
def main(cfg: DictConfig) -> None:
    """Main training function."""
    logger.info("Starting Pupper MJX RL Training")
    logger.info(f"Config:\n{OmegaConf.to_yaml(cfg)}")

    # System setup
    system_checker = SystemChecker()
    system_checker.check_gpu()
    system_checker.setup_environment()
    system_checker.verify_mujoco()

    # Model setup
    model_manager = ModelManager(cfg)
    model_manager.setup_repositories()
    model_path = model_manager.modify_model()

    # Training setup
    trainer = Trainer(cfg, model_path)
    trainer.setup_wandb()

    # Benchmark
    physics_steps_per_sec = trainer.benchmark_model()

    # Train
    make_inference_fn, params = trainer.train()

    # Export policy
    policy_path = trainer.export_policy(params)

    # Cleanup
    if trainer.wandb_run:
        wandb.finish()

    logger.info("Training completed successfully!")
    logger.info(f"Policy saved to: {policy_path}")


if __name__ == "__main__":
    main()
