# SwitchableTeleoperator 设计说明

## 目标

`SwitchableTeleoperator` 的目标是在不修改 `record.py` 和 `teleoperate.py` 现有 teleop 使用方式的前提下，为系统增加“一个 teleoperator 对外包装两个 teleoperator，并支持运行时切换”的能力。

外部调用方仍然只传入一个 `teleop` 配置。区别在于当 `teleop.type=switchable` 时，这个 `teleop` 本身是一个组合 teleoperator。

本次实现还引入了一个可扩展的键盘事件层，使键盘输入先转换为标准事件，再由 `SwitchableTeleoperator` 消费事件并执行切换逻辑。这样后续要增加其他键盘功能时，不需要改 `record` / `teleoperate` 主循环。

## 设计原则

1. 保持主控制入口不变
   `record.py` 和 `teleoperate.py` 继续只依赖 `Teleoperator` 抽象，不增加针对多 teleoperator 的专门分支。
2. 组合优先于侵入式修改
   用新的 `SwitchableTeleoperator` 实现组合能力，而不是改造现有所有 teleoperator 的接口。
3. 键盘监听与业务动作解耦
   键盘层只产出事件，不直接切换 teleoperator，也不直接生成机器人 action。
4. 为后续扩展预留空间
   v1 只实现切换主/备 teleoperator，但事件模型和绑定配置允许后续加入更多命令。

## 架构

### 1. SwitchableTeleoperatorConfig

新增配置类型 `SwitchableTeleoperatorConfig`，注册名为 `switchable`。

字段：

- `primary: TeleoperatorConfig`
- `secondary: TeleoperatorConfig`
- `keyboard_bindings: dict[str, str]`
- `default_active: str = "primary"`

说明：

- `primary` 和 `secondary` 是两个普通 teleoperator 的配置。
- `keyboard_bindings` 定义按键到命令名的映射，例如 `"1" -> "activate_primary"`。
- `default_active` 决定启动后默认从哪个 teleoperator 读取 action。

### 2. KeyboardEventSource

`KeyboardEventSource` 是键盘事件源，负责：

- 启动/停止键盘监听
- 将按键按下事件映射为 `TeleopEvent`
- 将事件放入队列，供外部消费

它不负责：

- 不切换 teleoperator
- 不改业务状态
- 不生成机器人 action

### 3. TeleopEvent

键盘层输出统一事件对象：

```python
TeleopEvent(type: str, payload: dict[str, Any] = {})
```

当前实现的事件类型：

- `activate_primary`
- `activate_secondary`

后续可以扩展：

- `toggle_active`
- `pause_output`
- `reset_active`
- 其它自定义控制事件

### 4. SwitchableTeleoperator

`SwitchableTeleoperator` 是标准 `Teleoperator` 子类，对外表现为一个普通 teleoperator，对内组合两个 teleoperator。

职责：

- 初始化并持有 `primary` / `secondary`
- 校验两个 teleoperator 的动作空间和反馈空间是否兼容
- 管理当前激活的 teleoperator
- 在 `get_action()` 前消费键盘事件
- 根据事件切换当前激活 teleoperator

运行逻辑：

1. `connect()`
   - 连接 `primary`
   - 连接 `secondary`
   - 启动 `KeyboardEventSource`
2. `get_action()`
   - drain 事件队列
   - 逐个调用 `handle_event(event)`
   - 从当前激活 teleoperator 读取 action
3. `send_feedback()`
   - 只发送给当前激活 teleoperator
4. `disconnect()`
   - 停止事件源
   - 断开两个 teleoperator

## 为什么采用事件层

如果直接把键盘监听和切换逻辑揉在一起，短期能工作，但后续会有几个问题：

1. 功能会写死在切换逻辑上
   将来新增键盘功能时，需要继续把业务逻辑堆进监听器。
2. 难复用
   其它 teleoperator 想用同样的键盘输入机制时，无法直接复用。
3. 难测试
   “按键处理”和“业务状态变更”混在一起时，单元测试不容易隔离。

事件层把系统拆成：

- 输入层：把按键翻译成事件
- 业务层：消费事件并执行动作

这样职责更清晰，也更方便以后替换输入源，例如游戏手柄事件、网络命令事件或 ROS 事件。

## 当前约束

本次实现刻意保持边界较小，当前有以下约束：

1. 只支持两个 teleoperator
2. `primary` 和 `secondary` 必须有相同的 `action_features`
3. `primary` 和 `secondary` 必须有相同的 `feedback_features`
4. 默认激活项只能是 `primary` 或 `secondary`
5. 如果环境不支持 `pynput`，但配置了键盘绑定，则连接阶段直接失败

这些约束是为了保证 `SwitchableTeleoperator` 仍然可以被外部当作一个普通 teleoperator 使用，不引入动作空间歧义。

## 配置示例

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

## 与现有系统的关系

本次接入只发生在 teleoperator 子系统内部：

- `src/lerobot/teleoperators/switchable/`
- `src/lerobot/teleoperators/utils.py`
- `src/lerobot/teleoperators/__init__.py`

`record.py` 和 `teleoperate.py` 没有增加新的多 teleop 逻辑分支，仍然只调用：

```python
teleop = make_teleoperator_from_config(cfg.teleop)
```

因此从控制入口视角看，`switchable` 只是新增的一种 teleoperator 类型。

## 后续扩展建议

可以沿着当前结构继续扩展：

1. 增加更多事件类型
   例如 `toggle_active`、`mute_feedback`、`hold_secondary`。
2. 支持更多输入源
   除键盘外，再加 gamepad 或网络事件源。
3. 支持多于两个 teleoperator
   将 `primary/secondary` 泛化为命名实例表，再让事件引用目标名称。
4. 增加状态可视化
   在日志或 rerun 中显示当前激活 teleoperator。
5. 支持事件回调或 hook
   便于某些切换动作触发同步反馈。

## 测试策略

当前测试重点覆盖：

1. 工厂是否能正确构造 `SwitchableTeleoperator`
2. 默认 action 是否来自 `primary`
3. 收到切换事件后 action 是否来自新的激活 teleoperator
4. 两个 teleoperator 的动作空间不一致时是否立即失败
5. 未识别事件是否被忽略而不是导致崩溃

对应测试文件：

- `tests/test_switchable_teleoperator.py`
