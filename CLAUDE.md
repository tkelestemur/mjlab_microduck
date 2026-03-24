# CLAUDE.md

## Project Overview

MjLab Microduck is a reinforcement learning project for training locomotion policies on the MicroDuck quadruped robot. It uses MuJoCo for physics simulation and the MJLab framework for RL training (PPO via RSL-RL). Trained policies can be exported to ONNX for deployment on the real robot.

Main hardware repo: https://github.com/apirrone/microduck

## Repository Structure

```
src/mjlab_microduck/
├── robot/
│   └── microduck/          # URDF, MuJoCo XMLs, mesh assets
│       ├── robot_walk.xml, robot_standup.xml, robot_ground_pick.xml
│       └── microduck_constants.py   # Joint names, default poses, limits
├── tasks/
│   ├── __init__.py                  # Task registration + MicroduckOnPolicyRunner
│   ├── mdp.py                       # Observations, rewards, terminations for velocity tasks
│   ├── imitation_mdp.py             # MDP functions for imitation tracking
│   ├── imitation_command.py         # ImitationCommand manager
│   ├── microduck_velocity_env_cfg.py
│   ├── microduck_standup_env_cfg.py
│   ├── microduck_ground_pick_env_cfg.py
│   └── microduck_imitation_env_cfg.py
├── data/
│   └── reference_motion.pkl         # Reference motions for imitation task
scripts/                             # Inference, replay, visualization utilities
export.py                            # ONNX model export
```

## Commands

All commands use `uv run` (UV package manager).

### Install dependencies
```
uv sync
```

### Train a policy
```
uv run train Mjlab-Velocity-Flat-MicroDuck --env.scene.num-envs 4096
```

### Resume training from checkpoint
```
uv run train Mjlab-Velocity-Flat-MicroDuck --env.scene.num-envs 4096 --agent.run-name resume --agent.load-checkpoint model_29999.pt --agent.resume True
```

### Play/visualize a trained policy
```
uv run play Mjlab-Velocity-Flat-MicroDuck --wandb-run-path <wandb-path>
```

### Export to ONNX
```
uv run export.py Mjlab-Velocity-Flat-MicroDuck --wandb-run-path <wandb-path>
```

### Lint
```
uv run ruff check src/
uv run ruff format src/
```

## Registered Tasks

| Task ID | Description |
|---------|-------------|
| `Mjlab-Velocity-Flat-MicroDuck` | Velocity-commanded walking on flat terrain |
| `Mjlab-Velocity-Rough-MicroDuck` | Velocity-commanded walking on rough terrain |
| `Mjlab-StandUp-Flat-MicroDuck` | Stand up from lying on back (flat) |
| `Mjlab-StandUp-Rough-MicroDuck` | Stand up from lying on back (rough) |
| `Mjlab-GroundPick-Flat-MicroDuck` | Crouch and touch ground with mouth (flat) |
| `Mjlab-GroundPick-Rough-MicroDuck` | Crouch and touch ground with mouth (rough) |
| `Mjlab-Imitation-Flat-MicroDuck` | Motion imitation tracking (requires reference_motion.pkl) |

## Key Architecture Patterns

- **Task registration**: Tasks are registered via `register_mjlab_task()` in `tasks/__init__.py` and discovered through the `mjlab.tasks` entry point in pyproject.toml.
- **MDP functions**: Observation, reward, and termination functions live in `mdp.py` (velocity/standup/ground-pick) and `imitation_mdp.py`. They operate on batched torch tensors across all environments.
- **Environment configs**: Each task has a `make_*_env_cfg()` factory function that returns the full environment configuration (observations, actions, rewards, terminations, curricula, domain randomization).
- **Custom runner**: `MicroduckOnPolicyRunner` extends `VelocityOnPolicyRunner` to sync `common_step_counter` on resume so curricula don't reset.
- **Domain randomization**: COM offsets, mass/inertia, Kp/Kd gains, joint friction, velocity pushes, IMU noise — all configured in env cfg files.

## Code Conventions

- Python 3.12+ required
- Ruff for linting and formatting (4-space indent)
- Reward functions return per-environment tensors
- Observation functions return per-environment tensors
- Each task has separate MuJoCo XML configs with task-specific collision groups
- Experiment tracking via Weights & Biases (wandb)
- `.pkl` files tracked via Git LFS

## Dependencies

- **mjlab**: Core RL framework (pinned to specific git rev in pyproject.toml)
- **mujoco**: Physics engine
- **torch**: Neural network training
- **onnx/onnxruntime**: Policy export and inference
- **wandb**: Experiment tracking

## Things to Know

- No automated test suite exists; validation is done by running training/play.
- No CI/CD pipeline is configured.
- The imitation task only registers if `reference_motion.pkl` exists on disk.
- Ghost visualization for imitation can be enabled via `GHOST=1` env var or `--ghost` flag.
- Generated artifacts (`.onnx`, `.pt`, `wandb/`, `logs/`) are gitignored.
