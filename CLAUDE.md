# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

LeRobot is a PyTorch-based robotics framework for imitation learning and reinforcement learning. It provides models, datasets, and tools for real-world robotics with a focus on lowering the barrier to entry for robotics research and development.

## Installation and Setup

### Standard Installation
```bash
git clone https://github.com/huggingface/lerobot.git
cd lerobot
conda create -y -n lerobot python=3.10
conda activate lerobot
conda install ffmpeg -c conda-forge
pip install -e .
```

### Development Setup (for contributors)
Using `poetry` or `uv` is recommended for development:
```bash
# Using poetry
poetry sync --extras "dev test"

# Using uv
uv sync --extra dev --extra test
```

Install pre-commit hooks:
```bash
pre-commit install
```

### Running Tests
```bash
# Run all tests
python -m pytest -sv ./tests

# Run specific test file
pytest tests/<test_file>.py

# Run with coverage
pytest tests/ --cov=lerobot
```

## Common Commands

### Training Policies
```bash
# Train using script with config from Hugging Face Hub
python -m lerobot.scripts.train --config_path=lerobot/diffusion_pusht

# Train with custom policy
python -m lerobot.scripts.train --policy.type=act --env.type=aloha --dataset.repo_id=lerobot/aloha_sim_transfer_cube_human
```

### Evaluating Policies
```bash
# Evaluate a pretrained policy
python -m lerobot.scripts.eval \
    --policy.path=lerobot/diffusion_pusht \
    --env.type=pusht \
    --eval.batch_size=10 \
    --eval.n_episodes=10

# Re-evaluate a trained checkpoint
python -m lerobot.scripts.eval --policy.path={OUTPUT_DIR}/checkpoints/last/pretrained_model
```

### Visualizing Datasets
```bash
# Visualize dataset from hub
python -m lerobot.scripts.visualize_dataset --repo-id lerobot/pusht --episode-index 0

# Visualize local dataset
python -m lerobot.scripts.visualize_dataset \
    --repo-id lerobot/pusht \
    --root ./my_local_data_dir \
    --local-files-only 1 \
    --episode-index 0
```

### End-to-End Tests (via Makefile)
```bash
# Run all E2E tests
make test-end-to-end

# Run specific policy tests
make test-act-ete-train
make test-diffusion-ete-eval
make test-tdmpc-ete-train

# Specify device (default: cpu)
make DEVICE=cuda test-end-to-end
```

### Docker
```bash
make build-cpu
make build-gpu
```

## High-Level Architecture

### Core Components

**Policies** (`src/lerobot/policies/`)
- ML model implementations: ACT, Diffusion, TDMPC, SmolVLA, SAC, VQ-BeT, Pi0
- Factory pattern for policy creation via `policies/factory.py`
- `get_policy_class(name)` - returns policy class by string name
- `make_policy(cfg, ds_meta, env_cfg)` - instantiates policies with proper normalization
- Config system uses `draccus` for YAML-based configuration
- Each policy has its own configuration class (e.g., `ACTConfig`, `DiffusionConfig`)

**Robots** (`src/lerobot/robots/`)
- Hardware abstraction for various robots: HopeJR, SO-100/SO-101, LeKiwi, Stretch3
- Motor control interfaces for different servos (Dynamixel, Feetech)
- Simulation and real-world interfaces

**Datasets** (`src/lerobot/datasets/`)
- `LeRobotDataset` format - custom efficient storage for robotics data
- Supports temporal observations with `delta_timestamps` (relative time offsets)
- Video storage in MP4 format for space efficiency
- HuggingFace hub integration for seamless upload/download
- Online buffer for real-time data collection
- Statistics caching (max, mean, min, std) for normalization

**Environments** (`src/lerobot/envs/`)
- Gymnasium-compatible environments
- Simulation (ALOHA, PushT, XArm) and real-world interfaces
- Policy evaluation environments

**Teleoperators** (`src/lerobot/teleoperators/`)
- Real-time teleoperation interfaces
- WebXR support for browser-based control
- Gamepad and exoskeleton integration

**Transport** (`src/lerobot/transport/`)
- Async inference services for distributed systems
- gRPC support with protocol buffer definitions

### Configuration System

LeRobot uses `draccus` for flexible YAML-based configuration management:
- Configs are defined as Python dataclasses in `src/lerobot/configs/`
- `TrainConfig`, `EvalConfig`, and policy-specific configs
- CLI override syntax: `--policy.learning_rate=0.001 --policy.device=cuda`
- Plugin system via `--env.discover_packages_path` for external extensions
- Config loading from HuggingFace Hub via `--config_path=repo_id`

### Data Flow

1. **Dataset Loading**: `LeRobotDataset` loads from hub or local path, provides temporal frame access via `delta_timestamps`
2. **Feature Processing**: `dataset_to_policy_features()` maps dataset features to policy input/output
3. **Policy Creation**: `make_policy()` instantiates policy with proper dimensions and normalization stats
4. **Training Loop**: Scripts in `src/lerobot/scripts/train.py` handle checkpointing, mixed precision, WandB logging
5. **Evaluation**: Scripts in `src/lerobot/scripts/eval.py` run policies on environments with configurable batch sizes

### Adding New Components

**New Policy:**
1. Create policy class in `src/lerobot/policies/<policy_name>/`
2. Set required `name` class attribute
3. Create configuration class in `<policy_name>/configuration_<policy_name>.py`
4. Update `available_policies` and `available_policies_per_env` in `src/lerobot/__init__.py`
5. Add entries to `tests/test_available.py`
6. Register in `policies/factory.py` in `get_policy_class()` and `make_policy_config()`

**New Dataset:**
1. Implement dataset compatible with LeRobotDataset format
2. Update `available_datasets_per_env` in `src/lerobot/__init__.py`

**New Environment:**
1. Create gymnasium-compatible environment
2. Update `available_tasks_per_env` and `available_datasets_per_env` in `src/lerobot/__init__.py`

## Code Style

- Line length: 110 characters
- Linter: `ruff` (configured in pyproject.toml)
- Pre-commit hooks: ruff formatting, typos, bandit security scanning
- Run `pre-commit run --all-files` to fix formatting on existing code

## Key Files to Understand

- `src/lerobot/policies/factory.py` - Policy instantiation pattern
- `src/lerobot/configs/parser.py` - Custom draccus wrapper with plugin loading
- `src/lerobot/scripts/train.py` - Training pipeline
- `src/lerobot/scripts/eval.py` - Evaluation pipeline
- `src/lerobot/__init__.py` - Registry of available components
