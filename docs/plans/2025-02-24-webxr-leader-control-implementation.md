# WebXR Leader Control Wrapper Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Create a `WebxrLeaderControlWrapper` that uses VR controllers (via WebXR) as a virtual leader device for HIL-SERL training.

**Architecture:** Extend `BaseLeaderControlWrapper` to treat WebXR as a virtual "leader robot," overriding methods that depend on physical hardware while maintaining the HIL-SERL intervention pattern.

**Tech Stack:** Python, Gymnasium, Zenoh messaging, WebXR VR APIs

---

## Prerequisites

### Understanding the Codebase

**Key Files to Review:**
- `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py` - Contains all wrapper classes and `make_robot_env()`
- `src/lerobot/teleoperators/webxr/teleop_webxr.py` - The `WebxrTeleop` class that provides VR controller data
- `src/lerobot/teleoperators/webxr/configuration_webxr.py` - Configuration for WebXR teleop

**Key Classes to Understand:**
- `BaseLeaderControlWrapper` (line ~1178) - Base class for leader-follower control
- `GearedLeaderControlWrapper` (line ~1455) - Physical leader implementation to reference
- `WebxrTeleop` - Teleoperator that receives VR controller poses via Zenoh

**Action Space Format:**
- Policy outputs: `[x, y, z, qx, qy, qz, qw, gripper]` (8 dimensions)
- `WebxrTeleop.get_action()` returns: `{x, y, z, qx, qy, qz, qw, gripper, mode, type}`

---

## Task 1: Create WebxrLeaderControlWrapper Class Skeleton

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Insert after `GearedLeaderAutomaticControlWrapper` class (around line 1505)

**Step 1: Add the class definition**

```python
class WebxrLeaderControlWrapper(BaseLeaderControlWrapper):
    """
    Wrapper for WebXR-based leader control in HIL-SERL training.

    Uses VR controllers (via WebXR/Zenoh) as a virtual leader device.
    Intervention is toggled via keyboard space key.

    Unlike physical leader wrappers, this wrapper:
    - Does not control any physical leader hardware
    - Does not synchronize leader-follower positions
    - Gets actions directly from WebXR pose data

    Args:
        env: The environment to wrap
        teleop_device: WebxrTeleop instance for VR controller input
        end_effector_step_sizes: Max step sizes for action clipping
        use_gripper: Whether gripper control is enabled
    """

    def __init__(self, env, teleop_device, end_effector_step_sizes, use_gripper=False):
        # Skip BaseLeaderControlWrapper's __init__ to avoid leader robot setup
        gym.Wrapper.__init__(self, env)
        self.teleop_device = teleop_device
        self.robot_follower = env.unwrapped.robot
        self.use_gripper = use_gripper
        self.end_effector_step_sizes = np.array(list(end_effector_step_sizes.values()))

        # Set up keyboard event tracking
        self._init_keyboard_events()
        self.event_lock = Lock()

        # Initialize kinematics for end-effector calculations
        from lerobot.utils.robot_utils import RobotKinematics, get_urdf_joint_names
        self.kinematics = RobotKinematics(
            urdf_path=env.unwrapped.robot.config.urdf_path,
            target_frame_name=env.unwrapped.robot.config.target_frame_name,
            joint_names=get_urdf_joint_names(env.unwrapped.robot),
        )

        # Previous WebXR pose for delta calculation
        self.prev_webxr_pos = None
        self.prev_webxr_quat = None

        # Initialize keyboard listener
        self._init_keyboard_listener()

        logger.info("WebxrLeaderControlWrapper initialized")
```

**Step 2: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 3: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: add WebxrLeaderControlWrapper class skeleton"
```

---

## Task 2: Override Keyboard Event Handling

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Inside `WebxrLeaderControlWrapper` class

**Step 1: Override _init_keyboard_events to add intervention toggle**

```python
    def _init_keyboard_events(self):
        """
        Initialize keyboard events dictionary.

        Extends base events with human_intervention_step flag for
        space key toggle.
        """
        self.keyboard_events = {
            "episode_success": False,
            "episode_end": False,
            "rerecord_episode": False,
            "human_intervention_step": False,
        }
```

**Step 2: Override _handle_key_press to add space key handling**

```python
    def _handle_key_press(self, key, keyboard_device):
        """
        Handle key press events for intervention control.

        Space key toggles intervention mode on/off.

        Args:
            key: The key that was pressed
            keyboard_device: The keyboard module with key definitions
        """
        try:
            if key == keyboard_device.Key.esc:
                self.keyboard_events["episode_end"] = True
                return
            if key == keyboard_device.Key.left:
                self.keyboard_events["rerecord_episode"] = True
                return
            if hasattr(key, "char") and key.char == "s":
                logging.info("Key 's' pressed. Episode success triggered.")
                self.keyboard_events["episode_success"] = True
                return
            if key == keyboard_device.Key.space:
                self.keyboard_events["human_intervention_step"] = not self.keyboard_events["human_intervention_step"]
                if self.keyboard_events["human_intervention_step"]:
                    log_say("WebXR intervention started", play_sounds=True)
                    logging.info("Space key pressed. WebXR intervention active.")
                else:
                    log_say("WebXR intervention ended", play_sounds=True)
                    logging.info("Space key pressed. Continuing with policy actions.")
        except Exception as e:
            logging.error(f"Error handling key press: {e}")
```

**Step 3: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 4: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: add keyboard event handling for WebXR intervention toggle"
```

---

## Task 3: Override _check_intervention Method

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Inside `WebxrLeaderControlWrapper` class

**Step 1: Add _check_intervention override**

```python
    def _check_intervention(self) -> bool:
        """
        Check if human intervention is active.

        Returns:
            True if space key has been pressed to enable intervention,
            False otherwise.

        Note: This overrides the automatic variance-based detection from
        BaseLeaderControlWrapper with manual keyboard toggle.
        """
        return self.keyboard_events["human_intervention_step"]
```

**Step 2: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 3: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: override _check_intervention for keyboard toggle"
```

---

## Task 4: Override _handle_leader_teleoperation as No-op

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Inside `WebxrLeaderControlWrapper` class

**Step 1: Add _handle_leader_teleoperation override**

```python
    def _handle_leader_teleoperation(self):
        """
        Handle leader teleoperation in non-intervention mode.

        For WebXR, there is no physical leader to synchronize.
        This is a no-op since WebXR only provides input during intervention.
        """
        # No-op: no physical leader robot to synchronize
        pass
```

**Step 2: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 3: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: override _handle_leader_teleoperation as no-op"
```

---

## Task 5: Implement _handle_intervention Method

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Inside `WebxrLeaderControlWrapper` class

**Step 1: Add _handle_intervention implementation**

```python
    def _handle_intervention(self, action) -> np.ndarray:
        """
        Calculate action from WebXR pose during intervention.

        Gets current pose from WebXR and calculates delta from previous pose.
        The delta is clipped and normalized for the action space.

        Args:
            action: The original action from the policy (ignored during intervention)

        Returns:
            np.ndarray: The action derived from WebXR controller movement
        """
        # Get current pose from WebXR
        webxr_action_dict = self.teleop_device.get_action()

        # Extract position and quaternion
        current_pos = np.array([
            webxr_action_dict["x"],
            webxr_action_dict["y"],
            webxr_action_dict["z"]
        ])
        current_quat = np.array([
            webxr_action_dict["qx"],
            webxr_action_dict["qy"],
            webxr_action_dict["qz"],
            webxr_action_dict["qw"]
        ])

        # Initialize previous pose on first intervention
        if self.prev_webxr_pos is None:
            self.prev_webxr_pos = current_pos
        if self.prev_webxr_quat is None:
            self.prev_webxr_quat = current_quat

        # Calculate position delta
        pos_delta = current_pos - self.prev_webxr_pos

        # Calculate rotation delta (simplified: use quaternion difference)
        # For small rotations, we can approximate with quaternion difference
        from scipy.spatial.transform import Rotation as R
        current_rot = R.from_quat(current_quat)
        prev_rot = R.from_quat(self.prev_webxr_quat)

        # Get relative rotation
        rel_rot = current_rot * prev_rot.inv()
        rot_delta = rel_rot.as_quat()  # [x, y, z, w]

        # Clip position delta to step sizes
        clipped_pos_delta = np.clip(
            pos_delta,
            -self.end_effector_step_sizes[:3],
            self.end_effector_step_sizes[:3]
        )

        # Normalize position delta to [-1, 1]
        normalized_pos_delta = clipped_pos_delta / self.end_effector_step_sizes[:3]

        # Build action array
        action_list = [
            normalized_pos_delta[0],
            normalized_pos_delta[1],
            normalized_pos_delta[2],
            rot_delta[0],
            rot_delta[1],
            rot_delta[2],
            rot_delta[3],
        ]

        # Add gripper action if enabled
        if self.use_gripper:
            gripper_val = webxr_action_dict.get("gripper", 1.0)
            action_list.append(float(gripper_val))

        # Update previous pose for next iteration
        self.prev_webxr_pos = current_pos
        self.prev_webxr_quat = current_quat

        return np.array(action_list, dtype=np.float32)
```

**Step 2: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 3: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: implement _handle_intervention for WebXR pose delta"
```

---

## Task 6: Add step() Method with Reset Logic

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** Inside `WebxrLeaderControlWrapper` class

**Step 1: Add step() method**

```python
    def step(self, action):
        """
        Execute a step with possible WebXR intervention.

        Args:
            action: The action from the policy

        Returns:
            Tuple of (observation, reward, terminated, truncated, info)
        """
        is_intervention = self._check_intervention()

        # Handle intervention or normal execution
        if is_intervention:
            action = self._handle_intervention(action)
        else:
            # Reset previous pose when not intervening
            self.prev_webxr_pos = None
            self.prev_webxr_quat = None

        # Step the environment
        obs, reward, terminated, truncated, info = self.env.step(action)

        if isinstance(action, np.ndarray):
            action = torch.from_numpy(action)

        # Add intervention info
        info["is_intervention"] = is_intervention
        info["action_intervention"] = action

        # Check for success or manual termination
        success = self.keyboard_events["episode_success"]
        terminated = terminated or self.keyboard_events["episode_end"] or success

        if success:
            reward = 1.0
            logging.info("Episode ended successfully with reward 1.0")

        return obs, reward, terminated, truncated, info
```

**Step 2: Add reset() method**

```python
    def reset(self, **kwargs):
        """
        Reset the environment and intervention state.

        Args:
            **kwargs: Keyword arguments passed to wrapped environment

        Returns:
            Tuple of (observation, info)
        """
        self.keyboard_events = dict.fromkeys(self.keyboard_events, False)
        self.prev_webxr_pos = None
        self.prev_webxr_quat = None
        return super().reset(**kwargs)
```

**Step 3: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 4: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: add step() and reset() methods to WebxrLeaderControlWrapper"
```

---

## Task 7: Register Control Mode in make_robot_env()

**Files:**
- Modify: `src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`

**Location:** In `make_robot_env()` function, around line 2284 (after `webxr` case)

**Step 1: Add leader_webxr control mode case**

Find the existing webxr case and add leader_webxr after it:

```python
    elif control_mode == "webxr":
        env = WebxrControlWrapper(
            env=env,
            teleop_device=teleop_device,
            end_effector_step_sizes=cfg.robot.end_effector_step_sizes,
            use_gripper=cfg.wrapper.use_gripper,
            auto_reset=True,
        )
    elif control_mode == "leader_webxr":
        assert isinstance(teleop_device, WebxrTeleop), (
            "teleop_device must be an instance of WebxrTeleop for leader_webxr control mode"
        )
        env = WebxrLeaderControlWrapper(
            env=env,
            teleop_device=teleop_device,
            end_effector_step_sizes=cfg.robot.end_effector_step_sizes,
            use_gripper=cfg.wrapper.use_gripper,
        )
    else:
        raise ValueError(f"Invalid control mode: {control_mode}")
```

**Step 2: Add WebxrTeleop import at top of file if not present**

Check that this import exists (around line 61-66):
```python
from lerobot.teleoperators import (
    gamepad,  # noqa: F401
    keyboard,  # noqa: F401
    webxr,
    make_teleoperator_from_config,
)
from lerobot.teleoperators.webxr.teleop_webxr import WebxrTeleop
```

**Step 3: Run syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 4: Commit**

```bash
git add src/lerobot/scripts/rl/supre_robot_gym_manipulator.py
git commit -m "feat: register leader_webxr control mode in make_robot_env"
```

---

## Task 8: Create Test Configuration File

**Files:**
- Create: `src/lerobot/configs/supre_robot_hil/env_config_supre_robot_follower_hil_leader_webxr.yaml`

**Step 1: Create configuration file**

```yaml
type: gym_manipulator

# ===============================================
# Robot (Follower) Configuration
# ===============================================
robot:
  type: sim_robot_hil
  urdf_path: "/home/smai/workspace/dikeke/webxr/supre_robot/rf2502_new_3_std.urdf"
  urdf_path1: "/home/smai/workspace/dikeke/webxr/supre_robot/rf2502_new_3_std.urdf"
  target_frame_name: arml_Link5
  end_effector_bounds:
    min:
      - -10.0
      - -10.0
      - -10.0
    max:
      - 10.0
      - 10.0
      - 10.0
  end_effector_step_sizes:
    x: 0.05
    y: 0.05
    z: 0.05

# ===============================================
# Teleoperation Configuration
# ===============================================
teleop:
  type: webxr
  zenoh_connect_key: "tcp/localhost:7447"
  zenoh_topic: "lerobot/webxr/teleop"
  use_gripper: true
  robot_type: "sim_robot"

# ===============================================
# Wrapper Configuration
# ===============================================
wrapper:
  display_cameras: false
  add_joint_velocity_to_observation: true
  add_current_to_observation: false
  add_ee_pose_to_observation: true
  crop_params_dict:
    observation.images.head_cam:
      - 270
      - 170
      - 90
      - 190
    observation.images.left_wrist_cam:
      - 0
      - 0
      - 480
      - 640
  resize_size:
    - 128
    - 128
  control_time_s: 60.0
  use_gripper: true
  gripper_quantization_threshold: null
  gripper_penalty: -0.02
  gripper_penalty_in_reward: false
  fixed_reset_joint_positions:
    - -30.0
    - -10.0
    - 10.0
    - 10.0
    - 10.0
    - 10.0
    - 1.0
  reset_time_s: 2.5
  control_mode: leader_webxr  # NEW: Use WebXR as leader device

# ===============================================
# Environment Configuration
# ===============================================
name: sim_robot_leader_webxr
mode: record
repo_id: "cocodee/hil"
dataset_root: "data/webxr_leader"
task: "grab things"
num_episodes: 10
episode: 0
pretrained_policy_name_or_path: null
device: cpu
push_to_hub: false
fps: 20

# ===============================================
# Feature Specifications
# ===============================================
features:
  observation.images.head_cam:
    type: VISUAL
    shape:
      - 3
      - 128
      - 128
  observation.images.left_wrist_cam:
    type: VISUAL
    shape:
      - 3
      - 128
      - 128
  observation.state:
    type: STATE
    shape:
      - 24
  action:
    type: ACTION
    shape:
      - 8

features_map:
  observation.images.head_cam: observation.images.head_cam
  observation.images.left_wrist_cam: observation.images.left_wrist_cam
  observation.state: observation.state
  action: action

reward_classifier_pretrained_path: null
```

**Step 2: Commit**

```bash
git add src/lerobot/configs/supre_robot_hil/env_config_supre_robot_follower_hil_leader_webxr.yaml
git commit -m "feat: add test configuration for leader_webxr control mode"
```

---

## Task 9: Create Integration Test

**Files:**
- Create: `tests/scripts/rl/test_webxr_leader_control_wrapper.py`

**Step 1: Create test file**

```python
"""
Tests for WebxrLeaderControlWrapper
"""
import pytest
import numpy as np
from unittest.mock import Mock, MagicMock, patch

from lerobot.scripts.rl.supre_robot_gym_manipulator import WebxrLeaderControlWrapper
from lerobot.teleoperators.webxr.teleop_webxr import WebxrTeleop


@pytest.fixture
def mock_webxr_teleop():
    """Create a mock WebxrTeleop device."""
    teleop = Mock(spec=WebxrTeleop)
    teleop.name = "webxr"

    # Default action return value
    teleop.get_action.return_value = {
        "x": 0.5,
        "y": 0.0,
        "z": 0.3,
        "qx": 0.0,
        "qy": 0.0,
        "qz": 0.0,
        "qw": 1.0,
        "gripper": 1.0,
        "mode": "IDLE",
        "type": "webxr"
    }
    return teleop


@pytest.fixture
def mock_env():
    """Create a mock gym environment."""
    env = Mock()
    env.unwrapped.robot = Mock()
    env.unwrapped.robot.config = Mock()
    env.unwrapped.robot.config.urdf_path = "/fake/path.urdf"
    env.unwrapped.robot.config.target_frame_name = "test_link"
    env.step.return_value = (
        {"observation": np.zeros(10)},
        0.0,
        False,
        False,
        {}
    )
    env.reset.return_value = ({"observation": np.zeros(10)}, {})
    return env


@pytest.fixture
def end_effector_step_sizes():
    """Standard step sizes for testing."""
    return {"x": 0.05, "y": 0.05, "z": 0.05}


def test_webxr_leader_control_wrapper_init(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that WebxrLeaderControlWrapper initializes correctly."""
    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    assert wrapper.teleop_device == mock_webxr_teleop
    assert wrapper.use_gripper is True
    assert wrapper.prev_webxr_pos is None
    assert wrapper.prev_webxr_quat is None


def test_check_intervention_default_false(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that intervention is false by default."""
    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    assert wrapper._check_intervention() is False


def test_check_intervention_after_space_key(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that space key toggles intervention."""
    from pynput import keyboard as keyboard_device

    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    # Simulate space key press
    wrapper._handle_key_press(keyboard_device.Key.space, keyboard_device)

    assert wrapper._check_intervention() is True


def test_handle_leader_teleoperation_is_noop(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that _handle_leader_teleoperation does nothing."""
    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    # Should not raise any errors
    wrapper._handle_leader_teleoperation()


def test_handle_intervention_returns_action(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that _handle_intervention returns properly shaped action."""
    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    action = wrapper._handle_intervention(np.zeros(8))

    # Should return 8-element array (x, y, z, qx, qy, qz, qw, gripper)
    assert action.shape == (8,)
    assert action.dtype == np.float32


def test_reset_clears_previous_pose(mock_env, mock_webxr_teleop, end_effector_step_sizes):
    """Test that reset clears previous pose tracking."""
    with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.RobotKinematics'):
        with patch('lerobot.scripts.rl.supre_robot_gym_manipulator.get_urdf_joint_names'):
            wrapper = WebxrLeaderControlWrapper(
                env=mock_env,
                teleop_device=mock_webxr_teleop,
                end_effector_step_sizes=end_effector_step_sizes,
                use_gripper=True
            )

    # Set some values
    wrapper.prev_webxr_pos = np.array([1.0, 2.0, 3.0])
    wrapper.prev_webxr_quat = np.array([0.0, 0.0, 0.0, 1.0])
    wrapper.keyboard_events["human_intervention_step"] = True

    wrapper.reset()

    assert wrapper.prev_webxr_pos is None
    assert wrapper.prev_webxr_quat is None
    assert wrapper.keyboard_events["human_intervention_step"] is False
```

**Step 2: Run tests**

Run: `pytest tests/scripts/rl/test_webxr_leader_control_wrapper.py -v`
Expected: Tests should pass (or fail with useful error messages to guide fixes)

**Step 3: Commit**

```bash
git add tests/scripts/rl/test_webxr_leader_control_wrapper.py
git commit -m "test: add integration tests for WebxrLeaderControlWrapper"
```

---

## Task 10: Manual Integration Test

**Files:**
- Create: `scripts/test_webxr_leader_manual.py`

**Step 1: Create manual test script**

```python
"""
Manual test script for WebXR Leader Control integration.

This script tests the full integration of WebxrLeaderControlWrapper
with a simulated robot environment.

Usage:
1. Start Zenoh router: `zenohd`
2. Open WebXR VR interface in browser
3. Run this script: `python scripts/test_webxr_leader_manual.py`
4. Press space key to toggle intervention
5. Move VR controller to move robot during intervention
"""
import logging
import sys

from lerobot.configs.parser import parse_hydra_config
from lerobot.scripts.rl.supre_robot_gym_manipulator import make_robot_env

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)


def main():
    """Main test function."""
    logger.info("Loading configuration...")
    cfg = parse_hydra_config([
        "src/lerobot/configs/supre_robot_hil/env_config_supre_robot_follower_hil_leader_webxr.yaml"
    ])

    logger.info("Creating environment...")
    env = make_robot_env(cfg)

    logger.info("=" * 60)
    logger.info("WebXR Leader Control Integration Test")
    logger.info("=" * 60)
    logger.info("")
    logger.info("Controls:")
    logger.info("  SPACE    - Toggle WebXR intervention on/off")
    logger.info("  ESC      - End episode")
    logger.info("  S        - Mark episode success")
    logger.info("  LEFT     - Rerecord episode")
    logger.info("")
    logger.info("Instructions:")
    logger.info("1. Press SPACE to start intervention")
    logger.info("2. Move VR controller to control robot")
    logger.info("3. Press SPACE again to stop intervention")
    logger.info("4. Robot will follow policy when intervention is off")
    logger.info("")

    try:
        obs, info = env.reset()
        logger.info(f"Environment reset. Observation keys: {list(obs.keys())}")

        episode_count = 0
        step_count = 0

        while episode_count < 5:  # Run 5 episodes
            # Dummy policy action (all zeros = no movement)
            action = env.action_space.sample()

            obs, reward, terminated, truncated, info = env.step(action)
            step_count += 1

            if step_count % 10 == 0:
                intervention_status = "ACTIVE" if info.get("is_intervention") else "inactive"
                logger.info(f"Step {step_count}: Intervention {intervention_status}")

            if terminated or truncated:
                episode_count += 1
                logger.info(f"Episode {episode_count} ended. Reward: {reward}")
                obs, info = env.reset()
                step_count = 0

    except KeyboardInterrupt:
        logger.info("Test interrupted by user")
    finally:
        env.close()
        logger.info("Environment closed")


if __name__ == "__main__":
    main()
```

**Step 2: Run manual test**

```bash
# Terminal 1: Start Zenoh router
zenohd

# Terminal 2: Run the manual test
python scripts/test_webxr_leader_manual.py
```

Expected:
- Environment loads without errors
- Space key toggles intervention
- VR controller movements generate actions

**Step 3: Commit**

```bash
git add scripts/test_webxr_leader_manual.py
git commit -m "test: add manual integration test script for WebXR leader control"
```

---

## Task 11: Update Documentation

**Files:**
- Modify: `docs/plans/2025-02-24-webxr-leader-control-design.md`

**Step 1: Add implementation notes section to design doc**

Add at the end of the design document:

```markdown
## Implementation Notes

### Usage

To use WebXR as a leader device in HIL-SERL training:

1. Configure environment with `control_mode: leader_webxr`
2. Set `teleop.type: webxr` with Zenoh connection details
3. Start training with HIL-SERL

Example configuration:
```yaml
teleop:
  type: webxr
  zenoh_connect_key: "tcp/localhost:7447"

wrapper:
  control_mode: leader_webxr
```

### Controls

- **SPACE**: Toggle WebXR intervention on/off
- **ESC**: End episode
- **S**: Mark episode as success
- **LEFT**: Rerecord episode

### Testing

Run unit tests:
```bash
pytest tests/scripts/rl/test_webxr_leader_control_wrapper.py -v
```

Run manual integration test:
```bash
python scripts/test_webxr_leader_manual.py
```

### Troubleshooting

**Problem**: Intervention doesn't activate when pressing space
**Solution**: Check that keyboard listener is running (no headless mode)

**Problem**: WebXR actions not applied
**Solution**: Verify Zenoh connection and check `teleop_device.get_action()` returns data

**Problem**: Robot moves too fast/slow during intervention
**Solution**: Adjust `end_effector_step_sizes` in config
```

**Step 2: Commit**

```bash
git add docs/plans/2025-02-24-webxr-leader-control-design.md
git commit -m "docs: add implementation notes to WebXR leader design"
```

---

## Task 12: Final Integration Verification

**Files:**
- All modified files

**Step 1: Full syntax check**

Run: `python -m py_compile src/lerobot/scripts/rl/supre_robot_gym_manipulator.py`
Expected: No syntax errors

**Step 2: Run all tests**

Run: `pytest tests/scripts/rl/test_webxr_leader_control_wrapper.py -v`
Expected: All tests pass

**Step 3: Verify import works**

```bash
python -c "
from lerobot.scripts.rl.supre_robot_gym_manipulator import WebxrLeaderControlWrapper
from lerobot.teleoperators.webxr.teleop_webxr import WebxrTeleop
print('Import successful!')
print(f'WebxrLeaderControlWrapper: {WebxrLeaderControlWrapper}')
print(f'WebxrTeleop: {WebxrTeleop}')
"
```

Expected: Import successful, classes printed

**Step 4: Verify control mode registration**

```bash
python -c "
from lerobot.configs.parser import parse_hydra_config
cfg = parse_hydra_config([
    'src/lerobot/configs/supre_robot_hil/env_config_supre_robot_follower_hil_leader_webxr.yaml'
])
print(f'Control mode: {cfg.wrapper.control_mode}')
print(f'Teleop type: {cfg.teleop.type}')
"
```

Expected: Control mode: `leader_webxr`, Teleop type: `webxr`

**Step 5: Final commit**

```bash
git add -A
git commit -m "feat: complete WebXR leader control wrapper implementation

- Add WebxrLeaderControlWrapper extending BaseLeaderControlWrapper
- Override _check_intervention, _handle_leader_teleoperation, _handle_intervention
- Register leader_webxr control mode in make_robot_env
- Add unit tests and manual integration test
- Add test configuration file
"
```

---

## Summary

This implementation plan creates a complete WebXR leader control integration:

1. **WebxrLeaderControlWrapper class** - Main wrapper handling VR-based intervention
2. **Keyboard toggle control** - Space key toggles intervention on/off
3. **Pose delta calculation** - Converts WebXR poses to action deltas
4. **Control mode registration** - `leader_webxr` in `make_robot_env()`
5. **Test suite** - Unit tests and manual integration test
6. **Documentation** - Design doc with implementation notes

The wrapper integrates seamlessly with the existing HIL-SERL training pipeline while treating WebXR as a virtual "leader robot" without physical hardware.
