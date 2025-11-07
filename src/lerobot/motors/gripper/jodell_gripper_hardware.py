import time
import jodell_gripper_py # 导入 pybind11 生成的模块
import threading # 导入 threading 模块
from ..eyou.hardware_interface import HardwareInterface
from lerobot.utils.monitor_utils import monitor_performance
import logging
# --- 辅助函数 (保持不变) ---

logger = logging.getLogger(__name__)

def convert_to_gripper_position(position_float: float) -> int:
    """将 0.0-1.0 范围的浮点数位置转换为 0-255 的整数。"""
    position_float = max(0.0, min(1.0, position_float))
    return int(position_float * 255.0)

def convert_from_gripper_position(position_uint8: int) -> float:
    """将 0-255 的整数位置转换为 0.0-1.0 范围的浮点数。"""
    return float(position_uint8) / 255.0

def convert_to_gripper_percentage(percentage: int) -> int:
    """将 0-100 的百分比转换为 0-255 的整数。"""
    percentage = max(0, min(100, percentage))
    return int(percentage * 255 / 100)

class JodellGripperHardware(HardwareInterface):
    """
    一个模仿 ros2_control HardwareInterface 风格的 Jodell 夹爪 Python 控制类。
    """

    def __init__(self):
        """构造函数，初始化所有成员变量。"""
        print("JodellGripperHardware: Initializing...")
        self.config = None
        self.gripper_bus = None
        self.gripper_clients = []
        
        self.slave_ids = []
        self.default_speed_percent = 50
        self.default_force_percent = 50
        
        # 模仿 ros2_control 的状态和命令向量
        self.hw_commands_position = []
        self.hw_states_position = []

        # --- 新增：线程相关成员 ---
        self._polling_thread = None
        self._stop_event = threading.Event()
        self._state_lock = threading.Lock() # 用于保护 hw_states_position 的读写
        self._polling_interval_seconds = 0.01 # 后台轮询所有夹爪的间隔，可根据总线速度调整
    def init(self, config: dict) -> bool:
        print("JodellGripperHardware: Running init...")
        self.config = config

        try:
            # 1. 获取硬件参数
            device = self.config["device"]
            baud_rate = self.config.get("baud_rate", 115200)
            self.default_speed_percent = self.config.get("default_speed_percent", 50)
            self.default_force_percent = self.config.get("default_torque_percent", 50)
            self._polling_interval_seconds = self.config.get("polling_interval_seconds", 0.02)
                        
            # 2. 验证并解析 "joints"
            if "joints" not in self.config or not self.config["joints"]:
                print("Error: Configuration must contain a non-empty 'joints' list.")
                return False

            num_joints = len(self.config["joints"])
            self.slave_ids = [0] * num_joints
            self.hw_commands_position = [None] * num_joints
            
            # 初始化状态向量
            with self._state_lock:
                self.hw_states_position = [0.0] * num_joints

            for i, joint_info in enumerate(self.config["joints"]):
                slave_id = int(joint_info["parameters"]["slave_id"])
                self.slave_ids[i] = slave_id
                print(f"  - Mapped joint '{joint_info['name']}' to slave_id {slave_id}")

        except (KeyError, ValueError, TypeError) as e:
            print(f"Error: Invalid or missing configuration parameter: {e}")
            return False
            
        print(f"Init successful. Found {len(self.slave_ids)} grippers to manage.")
        return True
    
    def activate(self) -> bool:
        # ... 此方法保持不变 ...
        if not self.config:
            print("Error: Must call init() with a valid configuration before activating.")
            return False
            
        print("JodellGripperHardware: Activating...")
        try:
            # 1. 创建并连接共享总线
            print(f"Connecting to bus on device '{self.config['device']}'...")
            self.gripper_bus = jodell_gripper_py.GripperBus(
                self.config["device"],
                self.config.get("baud_rate", 115200)
            )
            if not self.gripper_bus.connect():
                print("Error: Failed to connect to gripper bus.")
                self.gripper_bus = None
                return False
            print("Bus connected successfully.")

            # 2. 为每个 slave_id 创建 JodellGripper 客户端
            self.gripper_clients.clear()
            for slave_id in self.slave_ids:
                client = jodell_gripper_py.JodellGripper(self.gripper_bus, slave_id)
                self.gripper_clients.append(client)
            print(f"Created {len(self.gripper_clients)} gripper clients.")

            # 3. 激活所有夹爪
            for i, client in enumerate(self.gripper_clients):
                print(f"Enabling gripper with slave_id {self.slave_ids[i]}...")
                if not client.enable():
                    print(f"Error: Failed to enable gripper with slave_id {self.slave_ids[i]}.")
                    self.deactivate()
                    return False

            # --- 优化点：启动后台轮询线程 ---
            self._stop_event.clear()
            self._polling_thread = threading.Thread(target=self._polling_loop, daemon=True)
            self._polling_thread.start()
            print("Background polling thread started.")

            print("All grippers activated successfully.")
            return True

        except Exception as e:
            print(f"An exception occurred during activation: {e}")
            if self.gripper_bus and self.gripper_bus.is_connected():
                self.deactivate()
            return False
        
    def deactivate(self) -> bool:
        # ... 此方法保持不变 ...
        print("JodellGripperHardware: Deactivating...")
        if self._polling_thread:
            print("Stopping background polling thread...")
            self._stop_event.set()
            # 等待线程结束，设置一个超时以防万一
            self._polling_thread.join(timeout=1.0) 
            if self._polling_thread.is_alive():
                print("Warning: Polling thread did not terminate gracefully.")
            self._polling_thread = None
            print("Polling thread stopped.")

        for i, client in enumerate(self.gripper_clients):
            try:
                if not client.disable():
                     print(f"Warning: Failed to disable gripper with slave_id {self.slave_ids[i]}.")
            except Exception as e:
                print(f"Warning: Exception while disabling gripper {self.slave_ids[i]}: {e}")
        
        self.gripper_clients.clear()

        if self.gripper_bus:
            self.gripper_bus.disconnect()
            self.gripper_bus = None
            print("Bus disconnected.")
            
        return True    
    def _polling_loop(self):
        """
        后台线程执行的函数。它在一个循环中持续读取所有夹爪的状态。
        """
        while not self._stop_event.is_set():
            if not self.gripper_clients:
                # 如果尚未激活或已停用，则短暂休眠
                time.sleep(0.1)
                continue

            # 创建一个局部变量来存储本次轮询的结果
            local_states = [None] * len(self.slave_ids)
            for i, client in enumerate(self.gripper_clients):
                try:
                    # 这部分仍然是阻塞的，但它在后台线程中执行
                    status = client.get_status()
                    local_states[i] = convert_from_gripper_position(status.position)
                except RuntimeError as e:
                    # 在后台打印警告，不影响主线程
                    print(f"Polling Warning: Failed to read from slave {self.slave_ids[i]}: {e}")
                    local_states[i] = None
            
            # --- 关键：使用锁来安全地更新共享状态 ---
            with self._state_lock:
                self.hw_states_position = local_states
            
            # 在完成一轮完整的轮询后，稍作休息
            time.sleep(self._polling_interval_seconds)

    @monitor_performance
    def read(self) -> list[float | None]:
        """
        *** 优化后的非阻塞读取 ***
        从内存中快速获取由后台线程更新的最新夹爪位置。
        这个函数几乎是瞬间完成的。
        """
        start_t = time.perf_counter()
        with self._state_lock:
            # 返回一个副本，防止外部代码意外修改内部状态
            positions = self.hw_states_position.copy()
            logger.info(f"read time: {time.perf_counter() - start_t}")
            return positions
    # --- MODIFICATION ---
    def write(self, commands: list[float | None]) -> bool:
        """
        向所有夹爪写入新的位置命令，并更新内部命令状态。
        这个方法模仿了 C++ 版本的完整行为：
        1. 接收命令。
        2. 更新内部命令向量 `self.hw_commands_position`。
        3. 遍历内部命令向量，将命令发送到硬件。
        4. 发送后，重置内部命令向量的对应项为 None。

        :param commands: 一个浮点数列表，每个值代表要发送给对应夹爪的目标位置 (0.0-1.0)。
                         如果列表中的某个值为 None，则表示不向该夹爪发送新命令。
        :return: 如果所有命令都成功发送，返回 True，否则返回 False。
        """
        if len(commands) != len(self.gripper_clients):
            raise ValueError(
                f"Number of commands ({len(commands)}) does not match "
                f"number of grippers ({len(self.gripper_clients)})."
            )

        # 1. 更新内部命令向量，使其反映刚刚传入的命令
        self.hw_commands_position = commands

        all_success = True
        speed_8bit = convert_to_gripper_percentage(self.default_speed_percent)
        force_8bit = convert_to_gripper_percentage(self.default_force_percent)

        # 2. 遍历内部命令向量并发送到硬件
        for i, command in enumerate(self.hw_commands_position):
            if command is None:
                continue

            position_8bit = convert_to_gripper_position(command)
            
            try:
                client = self.gripper_clients[i]
                if not client.move(position_8bit, speed_8bit, force_8bit):
                    print(f"Warning: Failed to send move command to slave_id {self.slave_ids[i]}.")
                    all_success = False
            except RuntimeError as e:
                print(f"Warning: Exception while sending move command to slave_id {self.slave_ids[i]}: {e}")
                all_success = False
        
        # 3. 重置内部命令向量，以防下次循环时重复发送
        # 这精确地模仿了 C++ 版本中将 command 设置为 NaN 的行为
        self.hw_commands_position = [None] * len(self.gripper_clients)
        
        return all_success
    def get_joint_count(self) -> int:
        return len(self.slave_ids)

# --- 使用示例 (与第一个版本相同) ---
if __name__ == "__main__":
    gripper_config = {
        "device": "/dev/ttyTHS2",  # <-- !!! 修改这里 !!!
        "baud_rate": 115200,
        "default_speed_percent": 100,
        "default_torque_percent": 100,
        "joints": [
            {"name": "right_arm_joint_7", "parameters": {"slave_id": 17}},
        ]
    }

    gripper_hardware = JodellGripperHardware()

    try:
        if not gripper_hardware.init(config=gripper_config):
            exit(1)
        
        if not gripper_hardware.activate():
            exit(1)

        print("\n--- Gripper control active. Starting command loop. ---")
        time.sleep(1)

        print("Reading initial position...")
        initial_positions = gripper_hardware.read()
        print(f"Initial positions: {initial_positions}")
        time.sleep(1)

        num_grippers = len(gripper_config["joints"])

        # 命令1：完全打开所有夹爪 (位置 1.0)
        print("\nSending command: Open gripper(s)")
        open_commands = [1.0] * num_grippers
        gripper_hardware.write(open_commands)
        time.sleep(10) 
        current_positions = gripper_hardware.read()
        print(f"Positions after opening: {current_positions}")

        # 命令2：完全闭合所有夹爪 (位置 0.0)
        print("\nSending command: Close gripper(s)")
        close_commands = [0.0] * num_grippers
        gripper_hardware.write(close_commands)
        time.sleep(10)
        current_positions = gripper_hardware.read()
        print(f"Positions after closing: {current_positions}")
        
        # 命令3：移动到半开位置 (位置 0.5)
        print("\nSending command: Move to half position")
        half_open_commands = [0.5] * num_grippers
        gripper_hardware.write(half_open_commands)
        time.sleep(10)
        current_positions = gripper_hardware.read()
        print(f"Positions at half: {current_positions}")

    except Exception as e:
        print(f"\nAn error occurred in the main loop: {e}")

    finally:
        print("\n--- Deactivating hardware ---")
        gripper_hardware.deactivate()
        print("Program finished.")

