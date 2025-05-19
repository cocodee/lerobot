import enum
from typing import List, Optional, Any # For type hinting

class MockMitParam:
    """
    Mock for xyber::MitParam.
    Represents parameters for MIT control mode.
    """
    def __init__(self,
                 pos_min: float = 0.0, pos_max: float = 0.0,
                 vel_min: float = 0.0, vel_max: float = 0.0,
                 toq_min: float = 0.0, toq_max: float = 0.0,
                 kp_min: float = 0.0, kp_max: float = 0.0,
                 kd_min: float = 0.0, kd_max: float = 0.0):
        self.pos_min: float = pos_min
        self.pos_max: float = pos_max
        self.vel_min: float = vel_min
        self.vel_max: float = vel_max
        self.toq_min: float = toq_min
        self.toq_max: float = toq_max
        self.kp_min: float = kp_min
        self.kp_max: float = kp_max
        self.kd_min: float = kd_min
        self.kd_max: float = kd_max

    def __repr__(self):
        return (f"MockMitParam(pos_min={self.pos_min}, pos_max={self.pos_max}, ..., "
                f"kd_min={self.kd_min}, kd_max={self.kd_max})")

class MockActautorState(enum.Enum):
    """Mock for xyber::ActautorState."""
    STATE_DISABLE = 0
    STATE_ENABLE = 1
    STATE_CALIBRATION = 2

class MockActautorMode(enum.Enum):
    """Mock for xyber::ActautorMode."""
    MODE_CURRENT = 0
    MODE_CURRENT_RAMP = 1
    MODE_VELOCITY = 2
    MODE_VELOCITY_RAMP = 3
    MODE_POSITION = 4
    MODE_POSITION_RAMP = 5
    MODE_MIT = 6

class MockDcuImu:
    """
    Mock for xyber::DcuImu.
    Represents IMU data from a DCU.
    """
    def __init__(self, acc: Optional[List[float]] = None,
                 gyro: Optional[List[float]] = None,
                 quat: Optional[List[float]] = None):
        self._acc: List[float] = acc if acc is not None else [0.0, 0.0, 0.0]
        self._gyro: List[float] = gyro if gyro is not None else [0.0, 0.0, 0.0]
        self._quat: List[float] = quat if quat is not None else [0.0, 0.0, 0.0, 1.0] # Default to unit quaternion

        if len(self._acc) != 3:
            raise ValueError("acc must have 3 elements")
        if len(self._gyro) != 3:
            raise ValueError("gyro must have 3 elements")
        if len(self._quat) != 4:
            raise ValueError("quat must have 4 elements") # Corrected from 3 to 4 based on binding

    @property
    def acc(self) -> List[float]:
        return self._acc

    @acc.setter
    def acc(self, value: List[float]):
        if len(value) != 3:
            raise ValueError("Input list must have exactly 3 elements for acc")
        self._acc = [float(v) for v in value]

    @property
    def gyro(self) -> List[float]:
        return self._gyro

    @gyro.setter
    def gyro(self, value: List[float]):
        if len(value) != 3:
            raise ValueError("Input list must have exactly 3 elements for gyro")
        self._gyro = [float(v) for v in value]

    @property
    def quat(self) -> List[float]:
        return self._quat

    @quat.setter
    def quat(self, value: List[float]):
        if len(value) != 4: # Corrected from 3 to 4 based on binding
            raise ValueError("Input list must have exactly 4 elements for quat")
        self._quat = [float(v) for v in value]

    def __repr__(self):
        return f"MockDcuImu(acc={self.acc}, gyro={self.gyro}, quat={self.quat})"

class MockCtrlChannel(enum.Enum):
    """Mock for xyber::CtrlChannel."""
    CTRL_CH1 = 0
    CTRL_CH2 = 1
    CTRL_CH3 = 2

class MockActuatorType(enum.Enum):
    """Mock for xyber::ActuatorType."""
    POWER_FLOW_R86 = 0
    POWER_FLOW_R52 = 1
    POWER_FLOW_L28 = 2
    OMNI_PICKER = 3
    UNKOWN = 4 # Note: UNKOWN, not UNKNOWN, as per binding

class MockXyberController:
    """
    Mock for XyberController.
    Simulates the behavior of the Xyber robot controller.
    """
    _instance = None # For singleton pattern

    # Internal state for mock behavior
    _mock_dcumap: dict[str, dict] = {} # Stores DCU info like ethercat_id, imu_data, imu_offset_applied
    _mock_actuatormap: dict[str, dict] = {} # Stores actuator info like dcu_name, ch, type, can_id, state, mode, temp, power, effort, vel, pos, mit_param

    def __init__(self):
        # This mock might not strictly enforce the singleton pattern in the constructor
        # as pybind11's get_instance is a static method.
        print("MockXyberController initialized.")
        self._version_info = "MockVersion 1.0.0" # Example version
        self._is_started = False
        self._realtime_priority = 0
        self._bind_cpu = 0


    @staticmethod
    def get_instance():
        """Get or create XyberController instance."""
        if MockXyberController._instance is None:
            MockXyberController._instance = MockXyberController()
        return MockXyberController._instance

    def get_version(self) -> str:
        """Get the Version object (mocked as a string)."""
        print(f"Mock: get_version() -> {self._version_info}")
        return self._version_info

    def create_dcu(self, name: str, ethercat_id: int) -> None:
        """Create a dcu instance in XyberController."""
        if name in self._mock_dcumap:
            print(f"Mock Warning: DCU '{name}' already exists. Overwriting.")
        self._mock_dcumap[name] = {
            "ethercat_id": ethercat_id,
            "imu_data": MockDcuImu(), # Default IMU data
            "imu_offset_applied": False
        }
        print(f"Mock: create_dcu(name='{name}', ethercat_id={ethercat_id})")

    def attach_actuator(self, dcu_name: str,
                        ch: MockCtrlChannel = MockCtrlChannel.CTRL_CH1,
                        type: MockActuatorType = MockActuatorType.POWER_FLOW_R52,
                        actuator_name: str = "", # Default to empty string as per some conventions
                        can_id: int = 0) -> None:
        """Create an actuator instance and attach it on the dcu."""
        if not actuator_name: # If actuator_name is not provided or empty, generate one
            actuator_name = f"{dcu_name}_act_{ch.name}_{can_id}"

        if dcu_name not in self._mock_dcumap:
            raise ValueError(f"Mock Error: DCU '{dcu_name}' does not exist. Cannot attach actuator '{actuator_name}'.")
        if actuator_name in self._mock_actuatormap:
            print(f"Mock Warning: Actuator '{actuator_name}' already exists. Overwriting.")

        self._mock_actuatormap[actuator_name] = {
            "dcu_name": dcu_name,
            "ch": ch,
            "type": type,
            "can_id": can_id,
            "state": MockActautorState.STATE_DISABLE, # Default state
            "mode": MockActautorMode.MODE_POSITION, # Default mode
            "tempure": 25.0,
            "power_state": "NOMINAL", # Example
            "effort": 0.0,
            "velocity": 0.0,
            "position": 0.0,
            "mit_param": MockMitParam()
        }
        print(f"Mock: attach_actuator(dcu_name='{dcu_name}', ch={ch}, type={type}, actuator_name='{actuator_name}', can_id={can_id})")

    def set_realtime(self, rt_priority: int, bind_cpu: int) -> None:
        """Make realtime priority for ecat io thread."""
        self._realtime_priority = rt_priority
        self._bind_cpu = bind_cpu
        print(f"Mock: set_realtime(rt_priority={rt_priority}, bind_cpu={bind_cpu})")
        # In a real mock, you might log this or check if called before start()

    def start(self, ifname: str, cycle_ns: int, enable_dc: bool) -> None:
        """Start the ethercat communication."""
        if self._is_started:
            print("Mock Warning: start() called but already started.")
            return
        self._is_started = True
        print(f"Mock: start(ifname='{ifname}', cycle_ns={cycle_ns}, enable_dc={enable_dc})")
        # Simulate some startup delay or resource allocation if needed

    def stop(self) -> None:
        """Stop the ethercat communication."""
        if not self._is_started:
            print("Mock Warning: stop() called but not started.")
            return
        self._is_started = False
        print("Mock: stop()")

    def get_dcu_imu_data(self, name: str) -> MockDcuImu:
        """Get dcu report imu data."""
        if name not in self._mock_dcumap:
            raise ValueError(f"Mock Error: DCU '{name}' not found.")
        print(f"Mock: get_dcu_imu_data(name='{name}')")
        # Return a copy to prevent modification of internal state if DcuImu is mutable
        imu_data = self._mock_dcumap[name]["imu_data"]
        return MockDcuImu(acc=list(imu_data.acc), gyro=list(imu_data.gyro), quat=list(imu_data.quat))


    def apply_dcu_imu_offset(self, name: str) -> None:
        """Enable imu and apply offset."""
        if name not in self._mock_dcumap:
            raise ValueError(f"Mock Error: DCU '{name}' not found.")
        self._mock_dcumap[name]["imu_offset_applied"] = True
        print(f"Mock: apply_dcu_imu_offset(name='{name}') - IMU offset applied.")
        # Potentially modify self._mock_dcumap[name]["imu_data"] if offset application changes readings

    def _set_actuator_state(self, name: Optional[str], state: MockActautorState):
        if name:
            if name not in self._mock_actuatormap:
                raise ValueError(f"Mock Error: Actuator '{name}' not found.")
            self._mock_actuatormap[name]["state"] = state
            print(f"Mock: Actuator '{name}' state set to {state.name}")
        else: # All actuators
            for act_name in self._mock_actuatormap:
                self._mock_actuatormap[act_name]["state"] = state
            print(f"Mock: All actuators state set to {state.name}")

    def enable_all_actuator(self) -> None:
        """Enable all actuator."""
        self._set_actuator_state(None, MockActautorState.STATE_ENABLE)

    def enable_actuator(self, name: str) -> None:
        """Enable an actuator."""
        self._set_actuator_state(name, MockActautorState.STATE_ENABLE)

    def disable_all_actuator(self) -> None:
        """Disable all actuator."""
        self._set_actuator_state(None, MockActautorState.STATE_DISABLE)

    def disable_actuator(self, name: str) -> None:
        """Disable an actuator."""
        self._set_actuator_state(name, MockActautorState.STATE_DISABLE)

    def _get_actuator_value(self, name: str, key: str, default_val: Any = 0.0) -> Any:
        if name not in self._mock_actuatormap:
            raise ValueError(f"Mock Error: Actuator '{name}' not found.")
        val = self._mock_actuatormap[name].get(key, default_val)
        print(f"Mock: get_{key}(name='{name}') -> {val}")
        return val

    def get_tempure(self, name: str) -> float:
        """Get temperature."""
        return self._get_actuator_value(name, "tempure", 25.0)

    def get_power_state(self, name: str) -> str:
        """Get the Power State."""
        return self._get_actuator_value(name, "power_state", "UNKNOWN")

    def get_mode(self, name: str) -> MockActautorMode:
        """Get the Mode."""
        return self._get_actuator_value(name, "mode", MockActautorMode.MODE_POSITION)

    def get_effort(self, name: str) -> float:
        """Get the Torque."""
        return self._get_actuator_value(name, "effort")

    def get_velocity(self, name: str) -> float:
        """Get the Velocity."""
        return self._get_actuator_value(name, "velocity")

    def get_position(self, name: str) -> float:
        """Get the Position."""
        return self._get_actuator_value(name, "position")

    def set_mit_param(self, name: str, param: MockMitParam) -> None:
        """Set the Mit Param object."""
        if name not in self._mock_actuatormap:
            raise ValueError(f"Mock Error: Actuator '{name}' not found.")
        if not isinstance(param, MockMitParam):
            raise TypeError("param must be an instance of MockMitParam")
        self._mock_actuatormap[name]["mit_param"] = param # Store a copy if MitParam is mutable
        print(f"Mock: set_mit_param(name='{name}', param={param})")

    def set_mit_cmd(self, name: str, pos: float, vel: float, effort: float, kp: float, kd: float) -> None:
        """Set the PD cmd using MIT mode."""
        if name not in self._mock_actuatormap:
            raise ValueError(f"Mock Error: Actuator '{name}' not found.")
        
        actuator_data = self._mock_actuatormap[name]
        if actuator_data["state"] != MockActautorState.STATE_ENABLE:
            print(f"Mock Warning: set_mit_cmd called on disabled actuator '{name}'. Command may not be effective.")
        if actuator_data["mode"] != MockActautorMode.MODE_MIT:
             # Optionally, auto-switch mode or raise error
            print(f"Mock Info: Actuator '{name}' not in MODE_MIT. Switching to MODE_MIT.")
            actuator_data["mode"] = MockActautorMode.MODE_MIT
        
        # Simulate command application (e.g., update internal target, but actual position/velocity would change over time)
        actuator_data["target_pos_mit"] = pos
        actuator_data["target_vel_mit"] = vel
        actuator_data["target_effort_mit"] = effort
        actuator_data["target_kp_mit"] = kp
        actuator_data["target_kd_mit"] = kd
        
        # For simplicity, we can assume the command instantly affects the readable values in a basic mock
        # In a more complex mock, these would update based on a simulated physics loop.
        actuator_data["position"] = pos # Simplistic update
        actuator_data["velocity"] = vel # Simplistic update
        actuator_data["effort"] = effort # Simplistic update

        print(f"Mock: set_mit_cmd(name='{name}', pos={pos}, vel={vel}, effort={effort}, kp={kp}, kd={kd})")
        print(f"Mock: Actuator '{name}' internal targets updated. Current (simplified) state: pos={actuator_data['position']}, vel={actuator_data['velocity']}")

# Example usage (can be in a separate test file)
if __name__ == "__main__":
    print("--- Testing Mock Classes ---")

    # MitParam
    param = MockMitParam(pos_min=-1.0, pos_max=1.0, kp_max=100.0)
    print(param)

    # Enums
    print(MockActautorState.STATE_ENABLE)
    print(MockActautorMode.MODE_MIT)

    # DcuImu
    imu = MockDcuImu(acc=[0.1, 0.2, 9.8], quat=[0,0,0,1])
    print(imu)
    imu.gyro = [0.01, -0.02, 0.03]
    print(f"IMU gyro after update: {imu.gyro}")
    try:
        imu.acc = [1,2,3,4] # Should fail
    except ValueError as e:
        print(f"Caught expected error: {e}")


    # XyberController
    print("\n--- Testing MockXyberController ---")
    controller1 = MockXyberController.get_instance()
    controller2 = MockXyberController.get_instance()
    print(f"Controller instances are the same: {controller1 is controller2}")

    print(f"Version: {controller1.get_version()}")

    controller1.create_dcu("dcu_front", 0)
    controller1.create_dcu("dcu_rear", 1)

    controller1.attach_actuator(dcu_name="dcu_front", actuator_name="motor_fl", can_id=1, type=MockActuatorType.POWER_FLOW_R86)
    controller1.attach_actuator(dcu_name="dcu_front", actuator_name="motor_fr", can_id=2, ch=MockCtrlChannel.CTRL_CH2)
    try:
        controller1.attach_actuator(dcu_name="non_existent_dcu", actuator_name="motor_test")
    except ValueError as e:
        print(f"Caught expected error: {e}")


    controller1.set_realtime(rt_priority=80, bind_cpu=2)
    controller1.start(ifname="eth0", cycle_ns=1000000, enable_dc=True)

    front_imu_data = controller1.get_dcu_imu_data("dcu_front")
    print(f"Front DCU IMU: {front_imu_data}")
    front_imu_data.acc = [1,1,1] # Modifying the returned object
    # Re-fetch to see if internal state was modified (depends on how get_dcu_imu_data returns)
    # Our mock now returns a copy, so internal state should be unchanged by modifying the returned object.
    re_fetched_imu = controller1.get_dcu_imu_data("dcu_front")
    print(f"Re-fetched Front DCU IMU: {re_fetched_imu}") # Should be original if copy is returned

    controller1.apply_dcu_imu_offset("dcu_front")

    controller1.enable_actuator("motor_fl")
    controller1.enable_all_actuator() # Will re-enable motor_fl and enable motor_fr

    print(f"Motor FL position: {controller1.get_position('motor_fl')}")
    print(f"Motor FR mode: {controller1.get_mode('motor_fr').name}")

    new_mit_param = MockMitParam(kp_max=200.0, kd_max=10.0)
    controller1.set_mit_param("motor_fl", new_mit_param)

    controller1.set_mit_cmd("motor_fl", pos=1.57, vel=0.1, effort=0.5, kp=100.0, kd=5.0)
    print(f"Motor FL position after MIT cmd: {controller1.get_position('motor_fl')}") # Simplified update
    print(f"Motor FL mode after MIT cmd: {controller1.get_mode('motor_fl').name}")

    controller1.disable_actuator("motor_fr")
    controller1.disable_all_actuator()

    controller1.stop()

    print("\n--- Mock Internal State (for debugging/verification) ---")
    print("DCUs:", MockXyberController._mock_dcumap)
    print("Actuators:", MockXyberController._mock_actuatormap)