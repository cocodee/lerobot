# SwitchableTeleoperator 使用说明

## 功能简介

`SwitchableTeleoperator` 用来把两个普通 teleoperator 组合成一个新的 teleoperator。

对 `record.py` 和 `teleoperate.py` 来说，它仍然只是一个普通的 `teleop`，所以现有使用方式不需要改变。区别只是：

- 启动时会同时连接两个 teleoperator
- 运行时只使用当前激活的那个 teleoperator 输出 action
- 可以通过键盘事件切换当前激活的 teleoperator

## 适用场景

适合以下场景：

- 两个 leader arm 之间切换控制
- 两个不同来源的完整控制器之间切换
- 希望保留单 `teleop` 接口，但在运行时切换输入源
- 一个实体 leader arm 与一个基于 IK 的键盘关节 teleoperator 之间切换

不适合以下场景：

- 两个 teleoperator 同时合并输出 action
- 两个 teleoperator 的动作空间不一致
- 希望一个 teleoperator 控手臂、另一个 teleoperator 控底盘

这类“动作合成”需求不是 `SwitchableTeleoperator` 的目标。

## 与 `keyboard_ee` 的区别

仓库里原有的 `keyboard_ee` 适合 end-effector delta 控制链路，它输出的是：

- `delta_x`
- `delta_y`
- `delta_z`
- `gripper`

这种输出不能直接和 leader arm 这类关节空间 teleoperator 放进 `SwitchableTeleoperator`，因为 action key 不一致。

如果你需要和其它关节空间 teleoperator 切换，请使用新的 `keyboard_joint_ik`。它会：

- 用键盘控制末端平移和旋转
- 在 teleoperator 内部做 IK
- 直接输出关节空间 action，例如 `shoulder_pan.pos`

这样它就可以作为 `SwitchableTeleoperator` 的一个子 teleoperator 使用。

## `keyboard_joint_ik` 示例

下面是一个把实体 leader 与键盘 IK teleop 组合到一起的例子：

```yaml
teleop:
  type: switchable
  id: dual_input
  default_active: primary
  keyboard_bindings:
    "1": activate_primary
    "2": activate_secondary
  primary:
    type: so100_leader
    id: leader_a
    port: /dev/tty.usbmodemA
  secondary:
    type: keyboard_joint_ik
    id: keyboard_arm
    urdf_path: /path/to/so100.urdf
    output_mode: normalized
    key_bindings:
      "w": translate_y_negative
      "s": translate_y_positive
      "a": translate_x_positive
      "d": translate_x_negative
      "r": translate_z_positive
      "f": translate_z_negative
      "u": rotate_roll_negative
      "o": rotate_roll_positive
      "i": rotate_pitch_positive
      "k": rotate_pitch_negative
      "j": rotate_yaw_positive
      "l": rotate_yaw_negative
      "n": gripper_close
      "m": gripper_open
```

如果你的另一个 teleoperator 输出的是关节角度而不是归一化值，可以把 `output_mode` 改成 `degrees`。

## 配置方式

`SwitchableTeleoperator` 作为一种新的 teleoperator 类型使用，配置时设置：

```yaml
teleop:
  type: switchable
```

完整示例：

```yaml
teleop:
  type: switchable
  id: dual_input
  default_active: primary
  keyboard_bindings:
    "1": activate_primary
    "2": activate_secondary
  primary:
    type: so100_leader
    id: leader_a
    port: /dev/tty.usbmodemA
  secondary:
    type: so100_leader
    id: leader_b
    port: /dev/tty.usbmodemB
```

字段说明：

- `type`
  固定为 `switchable`
- `id`
  这个组合 teleoperator 自身的 id
- `default_active`
  启动后默认激活哪一个 teleoperator，可选 `primary` 或 `secondary`
- `keyboard_bindings`
  键盘按键到事件命令名的映射
- `primary`
  第一个 teleoperator 的配置
- `secondary`
  第二个 teleoperator 的配置

## 当前支持的命令

当前 `SwitchableTeleoperator` 内置支持以下事件命令：

- `activate_primary`
- `activate_secondary`

示例：

```yaml
keyboard_bindings:
  "1": activate_primary
  "2": activate_secondary
```

这表示：

- 按 `1` 切换到 `primary`
- 按 `2` 切换到 `secondary`

## 在 teleoperate 中使用

如果你原来是这样运行：

```bash
python -m lerobot.teleoperate --config_path=path/to/config.yaml
```

那么改成 `switchable` 配置后，运行方式不变，仍然是：

```bash
python -m lerobot.teleoperate --config_path=path/to/config.yaml
```

区别只是配置文件里的 `teleop` 从普通 teleoperator 换成了 `switchable`。

## 在 record 中使用

同样，录制方式也不变：

```bash
python -m lerobot.record --config_path=path/to/config.yaml
```

只要配置里的 `teleop.type` 改成 `switchable`，录制时就会使用组合 teleoperator。

## 运行时行为

启动后行为如下：

1. 连接 `primary`
2. 连接 `secondary`
3. 启动键盘事件监听
4. 按 `default_active` 指定的 teleoperator 作为当前控制源

每次控制循环里：

1. 先读取键盘事件
2. 如果有切换命令，就更新当前激活 teleoperator
3. 从当前激活 teleoperator 读取 action
4. 把 action 发给 robot

## 使用约束

当前版本有以下限制：

1. 只支持两个 teleoperator
2. 两个 teleoperator 的 `action_features` 必须完全一致
3. 两个 teleoperator 的 `feedback_features` 必须完全一致
4. 不支持把两个 teleoperator 的 action 合并
5. 不支持没有键盘环境却配置 `keyboard_bindings`

如果两个 teleoperator 动作空间不一致，初始化时会直接报错。

## 常见问题

### 1. 为什么不能一个 teleoperator 控机械臂、另一个控底盘？

因为 `SwitchableTeleoperator` 的设计目标是“独占切换”，不是“动作合成”。  
它要求两个 teleoperator 暴露相同动作空间，这样对外才能继续表现成一个标准 teleoperator。

### 2. 为什么配置了键盘绑定但切换没生效？

先检查以下几项：

- 当前环境是否支持 `pynput`
- Linux 下是否有 `DISPLAY`
- 终端是否有键盘监听权限
- 配置里的键和值是否正确

如果环境不支持键盘监听，`connect()` 阶段就应该直接报错。

### 3. 可以不配置键盘绑定吗？

可以。  
如果 `keyboard_bindings` 为空，`SwitchableTeleoperator` 仍然可以工作，只是运行时无法通过键盘切换，始终使用 `default_active` 指定的 teleoperator。

### 4. 可以配置别的命令吗？

配置上可以写任意非空命令名，但当前 `SwitchableTeleoperator` 只会处理：

- `activate_primary`
- `activate_secondary`

其它命令当前会被忽略，并记录 warning。这样做是为了给后续扩展留接口。

## 推荐配置模板

如果你只是想快速开始，建议从这个模板改：

```yaml
teleop:
  type: switchable
  id: my_switchable_teleop
  default_active: primary
  keyboard_bindings:
    "1": activate_primary
    "2": activate_secondary
  primary:
    type: so100_leader
    id: leader_primary
    port: /dev/tty.usbmodemXXXX
  secondary:
    type: so100_leader
    id: leader_secondary
    port: /dev/tty.usbmodemYYYY
```

## 相关文档

如果你需要了解内部实现和后续扩展方式，可以继续阅读：

- [`switchable_teleoperator_design.md`](/Users/kdi/workspace/gitprj/lerobot/lerobot/docs/switchable_teleoperator_design.md)
