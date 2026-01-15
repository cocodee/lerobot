import time
import misumi_gripper_py # 导入 PyBind11 生成的 Misumi 模块
from ..eyou.hardware_interface import HardwareInterface
# from lerobot.utils.monitor_utils import monitor_performance # 如果需要性能监控可取消注释
from typing import List, Dict, Any, Tuple, Optional

class MisumiGripperHardware(HardwareInterface):
    """
    基于 misumi_gripper_py 的 Misumi 夹爪 Python 控制类。
    实现了 HardwareInterface 接口，并模仿 ros2_control 风格。
    """

    def __init__(self):
        """构造函数，初始化所有成员变量。"""
        print("MisumiGripperHardware: Initializing...")
        self.config = None
        self.gripper_bus = None
        self.gripper_clients = []
        
        self.slave_ids = []
        self.default_speed_percent = 50
        self.default_torque_percent = 50
        
        # 行程参数，用于归一化 0.0-1.0
        self.min_position_mm = 0.0
        self.max_position_mm = 30.0 # 默认30mm，建议在 config 中配置
        
        # 状态和命令向量
        self.hw_commands_position = []
        self.hw_states_position = []
        self.hw_states_force = []    

        # 缓存相关变量
        self._cache_duration_seconds = 0.034  # 默认缓存34毫秒 (~30Hz)
        self._last_read_time = 0.0           # 上次真实读取的时间戳
        self._cached_values = []             # 缓存的位置数据

    def init(self, config: dict) -> bool:
        print("MisumiGripperHardware: Running init...")
        self.config = config

        try:
            # 1. 获取硬件参数
            # device, baud_rate 等
            self.default_speed_percent = self.config.get("default_speed_percent", 50)
            self.default_torque_percent = self.config.get("default_torque_percent", 50)
            self._cache_duration_seconds = self.config.get("cache_duration_seconds", 0.034)
            
            # --- Misumi 特有：行程配置 ---
            # 因为 Misumi 使用 mm 单位，我们需要知道最大行程来映射 0.0-1.0
            self.min_position_mm = self.config.get("min_position_mm", 0.0)
            self.max_position_mm = self.config.get("max_position_mm", 30.0) 
            
            if self.max_position_mm <= self.min_position_mm:
                print("Error: max_position_mm must be greater than min_position_mm.")
                return False
                        
            # 2. 验证并解析 "joints"
            if "joints" not in self.config or not self.config["joints"]:
                print("Error: Configuration must contain a non-empty 'joints' list.")
                return False

            num_joints = len(self.config["joints"])
            self.slave_ids = [0] * num_joints
            self.hw_commands_position = [None] * num_joints
            self.hw_states_position = [0.0] * num_joints
            self.hw_states_force = [0.0] * num_joints

            # 初始化缓存列表
            self._cached_values = [(None, None)] * num_joints

            for i, joint_info in enumerate(self.config["joints"]):
                slave_id = int(joint_info["parameters"]["slave_id"])
                self.slave_ids[i] = slave_id
                print(f"  - Mapped joint '{joint_info['name']}' to slave_id {slave_id}")
            
            print(f"  - Configured stroke: {self.min_position_mm}mm to {self.max_position_mm}mm")

        except (KeyError, ValueError, TypeError) as e:
            print(f"Error: Invalid or missing configuration parameter: {e}")
            return False
            
        print(f"Init successful. Found {len(self.slave_ids)} grippers to manage.")
        return True
    
    def activate(self) -> bool:
        if not self.config:
            print("Error: Must call init() with a valid configuration before activating.")
            return False
            
        print("MisumiGripperHardware: Activating...")
        try:
            # 1. 创建并连接共享总线
            # MisumiGripperBus 构造函数: device, baud_rate, parity='N', data_bit=8, stop_bit=1
            print(f"Connecting to bus on device '{self.config['device']}'...")
            self.gripper_bus = misumi_gripper_py.MisumiGripperBus(
                self.config["device"],
                self.config.get("baud_rate", 115200)
                # 可选参数 parity, data_bit, stop_bit 这里使用默认值
            )
            
            if not self.gripper_bus.connect():
                print(f"Error: Failed to connect to gripper bus. Last Error: {self.gripper_bus.getLastError()}")
                self.gripper_bus = None
                return False
            print("Bus connected successfully.")

            # 2. 为每个 slave_id 创建 MisumiGripper 客户端
            self.gripper_clients.clear()
            for slave_id in self.slave_ids:
                client = misumi_gripper_py.MisumiGripper(self.gripper_bus, slave_id)
                self.gripper_clients.append(client)
            print(f"Created {len(self.gripper_clients)} gripper clients.")

            # 3. 激活所有夹爪
            for i, client in enumerate(self.gripper_clients):
                print(f"Enabling gripper with slave_id {self.slave_ids[i]}...")
                if not client.enable():
                    print(f"Error: Failed to enable gripper {self.slave_ids[i]}. Last Error: {client.getLastError()}")
                    self.deactivate()
                    return False
            
            print("All grippers activated successfully.")
            return True

        except Exception as e:
            print(f"An exception occurred during activation: {e}")
            # 尝试清理资源
            if self.gripper_bus and self.gripper_bus.isConnected():
                self.deactivate()
            return False
        
    def deactivate(self) -> bool:
        print("MisumiGripperHardware: Deactivating...")
        for i, client in enumerate(self.gripper_clients):
            try:
                # Misumi disable
                if not client.disable():
                     print(f"Warning: Failed to disable gripper {self.slave_ids[i]}.")
            except Exception as e:
                print(f"Warning: Exception while disabling gripper {self.slave_ids[i]}: {e}")
        
        self.gripper_clients.clear()

        if self.gripper_bus:
            self.gripper_bus.disconnect()
            self.gripper_bus = None
            print("Bus disconnected.")
            
        return True    

    def read(self) -> List[Tuple[Optional[float], Optional[float]]]:
        """
        读取状态。如果距离上次读取时间小于缓存时间，直接返回缓存。
        """
        now = time.monotonic()
        
        # 1. 检查缓存是否有效
        if (now - self._last_read_time) < self._cache_duration_seconds:
            return self._cached_values

        # 2. 缓存失效，执行硬件读取
        if not self.gripper_clients:
            return [None] * len(self.slave_ids)

        range_mm = self.max_position_mm - self.min_position_mm

        for i, client in enumerate(self.gripper_clients):
            try:
                # 使用 PyBind 绑定的 pythonic 辅助函数 get_status
                # 返回 GripperStatus 对象或 None
                status = client.get_status()
                
                if status is not None:
                    # 转换 mm -> 0.0-1.0
                    pos_mm = status.position_mm
                    # 限制范围并归一化
                    clamped_mm = max(self.min_position_mm, min(self.max_position_mm, pos_mm))
                    normalized_pos = (clamped_mm - self.min_position_mm) / range_mm
                    self.hw_states_position[i] = float(normalized_pos)
                    
                    # 转换 torque % -> 0.0-1.0
                    self.hw_states_force[i] = float(status.torque_percent) / 100.0
                else:
                    print(f"Warning: Read failed for slave_id {self.slave_ids[i]} (get_status returned None)")
                    self.hw_states_position[i] = None
                    self.hw_states_force[i] = None
                    
            except RuntimeError as e:
                print(f"Warning: Exception reading status from slave_id {self.slave_ids[i]}: {e}")
                self.hw_states_position[i] = None
                self.hw_states_force[i] = None
        
        # 3. 更新缓存和时间戳
        self._cached_values = list(zip(self.hw_states_position, self.hw_states_force))
        self._last_read_time = now
        
        return list(zip(self.hw_states_position, self.hw_states_force))
    
    def write(self, commands: list[float | None]) -> bool:
        """
        写入位置命令 (0.0 - 1.0)。
        """
        if len(commands) != len(self.gripper_clients):
            raise ValueError(
                f"Number of commands ({len(commands)}) does not match "
                f"number of grippers ({len(self.gripper_clients)})."
            )

        self.hw_commands_position = commands

        all_success = True
        
        # 准备默认速度和力矩 (Int 百分比)
        speed_pct = int(max(0, min(100, self.default_speed_percent)))
        torque_pct = int(max(0, min(100, self.default_torque_percent)))
        
        range_mm = self.max_position_mm - self.min_position_mm

        for i, command in enumerate(self.hw_commands_position):
            if command is None:
                continue

            # 转换 0.0-1.0 -> mm
            command_clamped = max(0.0, min(1.0, command))
            target_mm = self.min_position_mm + (command_clamped * range_mm)
            
            try:
                client = self.gripper_clients[i]
                
                # Misumi API: moveTo(position_mm, speed_percent, torque_percent)
                # 注意：假设 PyBind 绑定如果底层返回 false/error 会抛出异常或返回 False
                # 根据 C++ 常见写法，如果绑定未指定返回值策略，通常返回 None。
                # 此处代码假设 moveTo 返回 bool 表示发送成功，或者无返回值(None)。
                result = client.moveTo(target_mm, speed_pct, torque_pct)
                
                # 如果 C++ 返回 void，result 为 None；如果返回 bool，result 为 True/False。
                # 兼容性处理：如果明确返回 False 才报错。
                if result is False: 
                     print(f"Warning: Failed to send moveTo command to slave_id {self.slave_ids[i]}.")
                     all_success = False

            except RuntimeError as e:
                print(f"Warning: Exception while sending move command to slave_id {self.slave_ids[i]}: {e}")
                all_success = False
        
        # 重置命令向量
        self.hw_commands_position = [None] * len(self.gripper_clients)
        
        return all_success

    def get_joint_count(self) -> int:
        return len(self.slave_ids)

# --- 使用示例 ---
if __name__ == "__main__":
    # 配置示例
    gripper_config = {
        "device": "/dev/ttyTHS1",  # Misumi 常见设备名
        "baud_rate": 115200,
        "default_speed_percent": 50,  # 50%
        "default_torque_percent": 50, # 50%
        # Misumi 特有配置：物理行程范围 (mm)
        "min_position_mm": 0.0,
        "max_position_mm": 40.0,      # 例如 Misumi GE-030 行程为 40mm
        "joints": [
            {"name": "gripper_joint", "parameters": {"slave_id": 27}},
        ]
    }

    gripper_hardware = MisumiGripperHardware()

    try:
        if not gripper_hardware.init(config=gripper_config):
            print("Initialization failed.")
            exit(1)
        
        if not gripper_hardware.activate():
            print("Activation failed.")
            exit(1)

        print("\n--- Gripper control active. Starting command loop. ---")
        time.sleep(1)

        print("Reading initial position...")
        initial_positions = gripper_hardware.read()
        print(f"Initial positions (0.0-1.0): {initial_positions}")
        time.sleep(1)

        num_grippers = len(gripper_config["joints"])

        # 命令1：完全打开 (1.0 -> max_position_mm)
        print("\nSending command: Open gripper(s) (1.0)")
        open_commands = [1.0] * num_grippers
        gripper_hardware.write(open_commands)
        time.sleep(3) # 等待运动
        current_positions = gripper_hardware.read()
        print(f"Positions after opening: {current_positions}")

        # 命令2：完全闭合 (0.0 -> min_position_mm)
        print("\nSending command: Close gripper(s) (0.0)")
        close_commands = [0.0] * num_grippers
        gripper_hardware.write(close_commands)
        time.sleep(3)
        current_positions = gripper_hardware.read()
        print(f"Positions after closing: {current_positions}")
        
        # 命令3：半开 (0.5)
        print("\nSending command: Move to half position (0.5)")
        half_open_commands = [0.5] * num_grippers
        gripper_hardware.write(half_open_commands)
        time.sleep(3)
        current_positions = gripper_hardware.read()
        print(f"Positions at half: {current_positions}")

    except KeyboardInterrupt:
        print("\nStopped by user.")
    except Exception as e:
        print(f"\nAn error occurred in the main loop: {e}")

    finally:
        print("\n--- Deactivating hardware ---")
        gripper_hardware.deactivate()
        print("Program finished.")