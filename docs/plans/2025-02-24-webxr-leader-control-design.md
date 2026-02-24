# WebXR Leader Control Wrapper Design

**Date:** 2025-02-24
**Status:** Design Approved
**Author:** Claude (with user input)

## Overview

This document describes the design for integrating `WebxrTeleop` as a "leader device" in the HIL-SERL training pipeline through a new `WebxrLeaderControlWrapper` class. This enables VR-based human intervention for reinforcement learning training without requiring physical leader robot hardware.

## Problem Statement

The existing `WebxrControlWrapper` treats WebXR as a simple input override mechanism. However, HIL-SERL training requires the leader-follower pattern provided by `BaseLeaderControlWrapper`:

1. Consistent intervention tracking in `info["is_intervention"]`
2. Action space that matches the expected HIL-SERL format
3. Integration with the existing control_mode infrastructure

## Proposed Solution

Create a new `WebxrLeaderControlWrapper` that extends `BaseLeaderControlWrapper` and treats WebXR as a virtual "leader robot."

### Architecture

```
┌─────────────────────────────────────────────────────────────────────┐
│                      WebxrLeaderControlWrapper                       │
├─────────────────────────────────────────────────────────────────────┤
│  ┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐ │
│  │  WebXR Client   │───▶│  WebxrTeleop    │───▶│  get_action()   │ │
│  │  (VR Browser)   │    │  (Zenoh Sub)    │    │  {x,y,z,q,g}    │ │
│  └─────────────────┘    └─────────────────┘    └─────────────────┘ │
│                                   │                                  │
│                                   ▼                                  │
│  ┌──────────────────────────────────────────────────────────────┐  │
│  │                    Step Flow                                  │  │
│  │  1. Check intervention (keyboard space key toggle)           │  │
│  │  2. If intervention active:                                  │  │
│  │     - Get action from WebXR pose delta                       │  │
│  │     - Apply clipped/normalized action                        │  │
│  │  3. Else: No-op (no leader sync needed)                      │  │
│  └──────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────────┐
│                    RobotEnv (Follower Robot)                        │
└─────────────────────────────────────────────────────────────────────┘
```

### Key Differences from Physical Leader

| Aspect | Physical Leader (`GearedLeaderControlWrapper`) | WebXR Leader (`WebxrLeaderControlWrapper`) |
|--------|-----------------------------------------------|-------------------------------------------|
| Hardware | Physical leader robot arm | VR controllers only |
| Position Source | `get_present_position(robot_leader)` | `webxr_teleop.get_action()` |
| Torque Control | Enable/disable leader torque | Not applicable (no hardware) |
| Leader Sync | Sync leader position to follower | Not needed (no hardware) |
| Intervention Trigger | Automatic (position variance) | Manual (space key toggle) |

## Class Design

### WebxrLeaderControlWrapper

```python
class WebxrLeaderControlWrapper(BaseLeaderControlWrapper):
    """
    Wrapper for WebXR-based leader control in HIL-SERL training.

    Uses VR controllers (via WebXR/Zenoh) as a virtual leader device.
    Intervention is toggled via keyboard space key.

    Args:
        env: The environment to wrap
        teleop_device: WebxrTeleop instance
        end_effector_step_sizes: Max step sizes for clipping
        use_gripper: Whether gripper is enabled
    """

    def __init__(self, env, teleop_device, end_effector_step_sizes, use_gripper=False):
        super().__init__(env, teleop_device, end_effector_step_sizes, use_gripper=use_gripper)

    def _check_intervention(self) -> bool:
        """Check intervention state via keyboard space key toggle."""
        return self.keyboard_events["human_intervention_step"]

    def _handle_leader_teleoperation(self):
        """No-op: no physical leader to synchronize."""
        pass

    def _handle_intervention(self, action) -> np.ndarray:
        """Calculate action from WebXR pose delta."""
        webxr_action = self.teleop_device.get_action()
        # Convert to action array with proper clipping and normalization
        ...
```

### Method Overrides

| Method | Override Type | Description |
|--------|--------------|-------------|
| `_check_intervention()` | Full | Return keyboard toggle state instead of automatic detection |
| `_handle_leader_teleoperation()` | No-op | No physical leader to sync |
| `_handle_intervention()` | Modified | Use WebXR pose delta instead of leader-follower delta |

## Data Flow

### Without Intervention (Policy Control)

```
Policy Action → RobotEnv.step() → Robot executes action
```

### With Intervention (WebXR Control)

```
WebXR Controller → Zenoh → WebxrTeleop.get_action()
                                        │
                                        ▼
                     {x, y, z, qx, qy, qz, qw, gripper}
                                        │
                                        ▼
                     WebxrLeaderControlWrapper calculates delta
                                        │
                                        ▼
                     Clip and normalize action
                                        │
                                        ▼
                     RobotEnv.step(modified_action)
```

## Configuration

### Environment Config

```yaml
type: gym_manipulator

robot:
  type: sim_robot_hil  # or supre_robot_follower_hil for real robot
  # ... robot config ...

teleop:
  type: webxr
  zenoh_connect_key: "tcp/localhost:7447"
  zenoh_topic: "lerobot/webxr/teleop"
  use_gripper: true

wrapper:
  control_mode: leader_webxr  # New control mode
  use_gripper: true
  # ... other wrapper config ...
```

### Integration in make_robot_env()

```python
elif control_mode == "leader_webxr":
    assert isinstance(teleop_device, WebxrTeleop), (
        "teleop_device must be WebxrTeleop for leader_webxr control mode"
    )
    env = WebxrLeaderControlWrapper(
        env=env,
        teleop_device=teleop_device,
        end_effector_step_sizes=cfg.robot.end_effector_step_sizes,
        use_gripper=cfg.wrapper.use_gripper,
    )
```

## Implementation Plan

1. **Create WebxrLeaderControlWrapper class**
   - Extend `BaseLeaderControlWrapper`
   - Override `_check_intervention()`, `_handle_leader_teleoperation()`, `_handle_intervention()`

2. **Register control mode in make_robot_env()**
   - Add `leader_webxr` case
   - Add type assertion for `WebxrTeleop`

3. **Test integration**
   - Verify intervention toggle works
   - Verify WebXR actions are properly applied
   - Verify HIL-SERL training integration

## Future Enhancements

- [ ] VR controller button trigger for intervention (instead of keyboard)
- [ ] Automatic intervention detection based on VR controller motion
- [ ] Visual feedback in VR when intervention is active
- [ ] Haptic feedback on intervention state change

## References

- `BaseLeaderControlWrapper`: `/Users/kdi/workspace/gitprj/lerobot/lerobot/src/lerobot/scripts/rl/supre_robot_gym_manipulator.py:1178`
- `WebxrTeleop`: `/Users/kdi/workspace/gitprj/lerobot/lerobot/src/lerobot/teleoperators/webxr/teleop_webxr.py`
- HIL-SERL: https://hil-serl.github.io/
