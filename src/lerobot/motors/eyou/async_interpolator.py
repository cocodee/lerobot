import queue
import threading
import time
from typing import List, Dict, Any, Optional
from .hardware_interface import HardwareInterface
# 假设 HardwareInterface 已定义
# from hardware_interface import HardwareInterface 

class AsyncInterpolator(HardwareInterface):
    """
    一个实现了 HardwareInterface 的装饰器类。
    它接收一个基础的 HardwareInterface 对象，并为其添加异步插值功能。
    对于外部调用者来说，它看起来就像一个普通的硬件对象。
    """

    def __init__(self, hardware: HardwareInterface, config: Dict[str, Any]):
        """
        构造函数。
        :param hardware: 一个基础的、实现了 HardwareInterface 的硬件对象。
        :param config: 包含插值器控制参数的配置字典。
        """
        if not isinstance(hardware, HardwareInterface):
            raise TypeError("hardware object must implement HardwareInterface")
        
        # 1. 保存被装饰的（底层的）硬件对象
        self._base_hardware: HardwareInterface = hardware
        self._config = config
        
        interpolation_n = int(self._config.get("interpolation_n", 2))
        self._interpolation_enabled = interpolation_n > 1

        if self._interpolation_enabled:
            print(f"AsyncInterpolator: Interpolation is ENABLED (interpolation_n = {interpolation_n}).")
            # 只有在启用时才初始化异步相关组件
            self._command_queue = queue.Queue(maxsize=1)
            self._writer_thread: Optional[threading.Thread] = None
            self._stop_event = threading.Event()
            
            control_frequency = float(self._config.get("control_frequency", 30.0))
            self._writer_frequency = control_frequency * interpolation_n
            self._interp_duration = 1.0 / control_frequency
            
            self._interp_start_pos: List[float] = []
            self._interp_end_pos: List[float] = []
            self._interp_start_time: float = 0.0
        else:
            print("AsyncInterpolator: Interpolation is DISABLED (interpolation_n <= 1). Operating in pass-through mode.")
            # 确保这些成员存在但为 None，避免后续代码出错
            self._command_queue = None
            self._writer_thread = None
            self._stop_event = None

    # --- 实现 HardwareInterface 的方法 ---

    def init(self, config: Dict[str, Any]) -> bool:
        """
        初始化底层的硬件。
        注意：config 在这里是给底层硬件的，而不是给插值器的。
        插值器的配置在 __init__ 时已经传入。
        """
        return self._base_hardware.init(config)

    def activate(self) -> bool:
        """激活底层硬件，然后启动插值线程。"""
        print("AsyncInterpolator: Activating...")
        if not self._base_hardware.activate():
            print("AsyncInterpolator Error: Base hardware activation failed.")
            return False
        
        # --- MODIFICATION: 只有在启用插值时才启动线程 ---
        if self._interpolation_enabled:
            initial_pos = self._base_hardware.read()
            if not initial_pos or any(p is None for p in initial_pos):
                print("AsyncInterpolator Error: Failed to read valid initial positions.")
                self._base_hardware.deactivate()
                return False
                
            self._interp_start_pos = list(initial_pos)
            self._interp_end_pos = list(initial_pos)
            self._interp_start_time = time.monotonic()

            self._stop_event.clear()
            self._writer_thread = threading.Thread(target=self._writer_loop, daemon=True)
            self._writer_thread.start()
        
        print("AsyncInterpolator: Activated successfully.")
        return True

    def deactivate(self):
        """停止插值线程，然后停用底层硬件。"""
        # --- MODIFICATION: 只有在启用插值时才停止线程 ---
        if self._interpolation_enabled and self._writer_thread and self._writer_thread.is_alive():
            self._stop_event.set()
            # 在队列中放入一个虚拟项来唤醒可能阻塞的 get()
            if self._command_queue:
                try: self._command_queue.put_nowait([]) 
                except queue.Full: pass
            self._writer_thread.join(timeout=1.0)
        
        self._base_hardware.deactivate()
        print("AsyncInterpolator: Deactivated.")

    def read(self) -> List[float]:
        """
        直接从底层硬件读取数据。
        这确保了主循环总能得到最新的、未经插值的反馈。
        """
        print(f"AsyncInterpolator: Reading...")
        print(f"AsyncInterpolator: Read: {self._base_hardware.read()}")
        return self._base_hardware.read()

    def write(self, commands_positions: List[float]):
        """
        如果插值已启用，将命令放入队列。
        如果插值已禁用，直接将命令传递给底层硬件。
        """
        # --- MODIFICATION: 根据标志选择行为 ---
        if self._interpolation_enabled:
            # 行为1：异步插值
            try:
                # 清空旧命令，只保留最新的
                while not self._command_queue.empty():
                    self._command_queue.get_nowait()
                self._command_queue.put_nowait(commands_positions)
            except (queue.Full, AttributeError):
                # AttributeError: self._command_queue is None
                pass
        else:
            # 行为2：直接写入
            self._base_hardware.write(commands_positions)
            
    def get_joint_count(self) -> int:
        """从底层硬件获取关节数量。"""
        return self._base_hardware.get_joint_count()

    # --- 内部的写入线程 (消费者) ---
    def _writer_loop(self):
        # ... (这个方法的内部逻辑与之前完全相同) ...
        # 它使用 self._base_hardware.read() 和 self._base_hardware.write()
        # 来与真实硬件交互。
        print("Writer thread started.")
        period = 1.0 / self._writer_frequency
        while not self._stop_event.is_set():
            loop_start_time = time.perf_counter()
            try:
                new_target = self._command_queue.get_nowait()
                current_hw_pos = self._base_hardware.read()
                self._interp_start_pos = current_hw_pos
                self._interp_end_pos = new_target
                self._interp_start_time = time.monotonic()
            except queue.Empty:
                pass
            
            now = time.monotonic()
            time_since_start = now - self._interp_start_time
            alpha = min(1.0, max(0.0, time_since_start / self._interp_duration))
            interpolated_positions = [s + alpha * (e - s) for s, e in zip(self._interp_start_pos, self._interp_end_pos)]
            
            # **注意**: 这里调用的是底层硬件的 write 方法！
            self._base_hardware.write(interpolated_positions)

            loop_end_time = time.perf_counter()
            sleep_time = period - (loop_end_time - loop_start_time)
            if sleep_time > 0:
                time.sleep(sleep_time)
                print(f"AsyncInterpolator: Writer thread loop completed in {loop_end_time - loop_start_time:.4f} seconds.")
        print("Writer thread stopped.")