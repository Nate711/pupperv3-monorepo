#!/usr/bin/env python3
"""Test script to validate Hydra configuration."""

import sys
from pathlib import Path

try:
    import hydra
    from omegaconf import DictConfig, OmegaConf
except ImportError as e:
    print(f"Missing dependencies: {e}")
    print("Please install requirements: pip install hydra-core omegaconf")
    sys.exit(1)


@hydra.main(version_base=None, config_path="conf", config_name="config")
def test_config(cfg: DictConfig) -> None:
    """Test configuration loading."""
    print("✅ Hydra configuration loaded successfully!")
    print("\nConfiguration structure:")
    print(OmegaConf.to_yaml(cfg))
    
    # Test accessing nested configs
    try:
        print(f"\n✅ Simulation config: {cfg.simulation.model_repo}")
        print(f"✅ Training timesteps: {cfg.training.ppo.num_timesteps:,}")
        print(f"✅ Policy architecture: {cfg.policy.hidden_layer_sizes}")
        print(f"✅ Reward scales available: {len(cfg.reward.rewards.scales)} types")
    except Exception as e:
        print(f"❌ Error accessing config values: {e}")
        return
    
    print("\n✅ All configuration tests passed!")


if __name__ == "__main__":
    test_config()