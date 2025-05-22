import unittest
from unittest.mock import MagicMock, patch, call, ANY
import numpy as np
import logging

# Make sure the lerobot module is accessible, adjust path if necessary
from lerobot.common.robot_devices.motors.agibotx1 import AgibotX1MotorsBus
from lerobot.common.robot_devices.motors.configs import AgibotX1MotorsBusConfig
# Attempt to import enums; if xyber_controller_py is not available, create Mocks
try:
    from lerobot.common.robot_devices.motors.xyber_controller_py import ActuatorType, CtrlChannel
except ImportError:
    # Create mock enums if the real ones can't be imported (e.g. in CI without hardware deps)
    class MockEnum: # pragma: no cover
        def __init__(self, name, value):
            self.name = name
            self.value = value
        def __eq__(self, other): 
            if isinstance(other, MockEnum):
                return self.name == other.name and self.value == other.value
            return NotImplemented
        def __hash__(self): 
            return hash((self.name, self.value))

    class ActuatorType: # pragma: no cover
        POWER_FLOW_R86 = MockEnum("POWER_FLOW_R86", 0)
        POWER_FLOW_R52 = MockEnum("POWER_FLOW_R52", 1)
        POWER_FLOW_L28 = MockEnum("POWER_FLOW_L28", 2)
        OMNI_PICKER = MockEnum("OMNI_PICKER", 3)

    class CtrlChannel: # pragma: no cover
        CTRL_CH1 = MockEnum("CTRL_CH1", 1)
        CTRL_CH2 = MockEnum("CTRL_CH2", 2)
        CTRL_CH3 = MockEnum("CTRL_CH3", 3)


from lerobot.common.robot_devices.utils import RobotDeviceAlreadyConnectedError, RobotDeviceNotConnectedError
from lerobot.common.robot_devices.motors.agibotx1 import (
    MIT_CMD_DEFAULT_POS, 
    MIT_CMD_DEFAULT_VEL, 
    MIT_CMD_DEFAULT_EFF,
    DATA_NAME_POSITION,
    DATA_NAME_VELOCITY,
    DATA_NAME_EFFORT,
    CALIBRATION_REQUIRED
)

def to_dcu_actuator_type_test_helper(actuator_type_str): # pragma: no cover
    if actuator_type_str == "POWER_FLOW_R86": return ActuatorType.POWER_FLOW_R86
    if actuator_type_str == "POWER_FLOW_R52": return ActuatorType.POWER_FLOW_R52
    if actuator_type_str == "POWER_FLOW_L28": return ActuatorType.POWER_FLOW_L28
    if actuator_type_str == "OMNI_PICKER": return ActuatorType.OMNI_PICKER
    raise ValueError(f"Invalid actuator type for test: {actuator_type_str}")

def to_dcu_channel_test_helper(ch_int): # pragma: no cover
    if ch_int == 1: return CtrlChannel.CTRL_CH1
    if ch_int == 2: return CtrlChannel.CTRL_CH2
    if ch_int == 3: return CtrlChannel.CTRL_CH3
    raise ValueError(f"Invalid channel for test: {ch_int}")


class TestAgibotX1MotorsBus(unittest.TestCase):
    def setUp(self):
        self.motors_config_dict = {
            "motor_1": [1, 11, "POWER_FLOW_R86"], 
            "motor_2": [2, 12, "POWER_FLOW_R52"],
        }
        self.config = AgibotX1MotorsBusConfig(
            dcu_name="test_dcu",
            ethercat_id=1,
            if_name="eth_test",
            motors=self.motors_config_dict,
            mock=False 
        )

        self.mock_xyber_controller_instance = MagicMock()
        
        self.patcher_get_controller = patch('lerobot.common.robot_devices.motors.agibotx1.get_controller')
        self.mock_get_controller_factory = self.patcher_get_controller.start()
        self.mock_get_controller_factory.return_value = self.mock_xyber_controller_instance
        
        self.patcher_to_dcu_act_type = patch('lerobot.common.robot_devices.motors.agibotx1.to_dcu_actuator_type', side_effect=to_dcu_actuator_type_test_helper)
        self.mock_to_dcu_act_type = self.patcher_to_dcu_act_type.start()
        
        self.patcher_to_dcu_channel = patch('lerobot.common.robot_devices.motors.agibotx1.to_dcu_channel', side_effect=to_dcu_channel_test_helper)
        self.mock_to_dcu_channel = self.patcher_to_dcu_channel.start()

        self.bus = AgibotX1MotorsBus(self.config)
        self.bus.is_connected = False 

    def tearDown(self):
        self.patcher_get_controller.stop()
        self.patcher_to_dcu_act_type.stop()
        self.patcher_to_dcu_channel.stop()

    # 1. connect Method Tests
    def test_connect_already_connected(self):
        self.bus.is_connected = True 
        with self.assertRaisesRegex(RobotDeviceAlreadyConnectedError, 
                                     f"AgibotX1MotorsBus for DCU '{self.config.dcu_name}' is already connected."):
            self.bus.connect()
        self.mock_xyber_controller_instance.attach_actuator.assert_not_called()

    def test_connect_success(self):
        self.mock_xyber_controller_instance.attach_actuator.return_value = True 
        self.bus.connect() 
        self.assertTrue(self.bus.is_connected) 
        expected_calls = []
        for name, params in self.motors_config_dict.items():
            expected_calls.append(
                call( 
                    dcu_name=self.config.dcu_name,
                    ch=to_dcu_channel_test_helper(params[0]),
                    type=to_dcu_actuator_type_test_helper(params[2]),
                    actuator_name=name,
                    can_id=params[1]
                )
            )
        self.mock_xyber_controller_instance.attach_actuator.assert_has_calls(expected_calls, any_order=False)
        self.assertEqual(self.mock_xyber_controller_instance.attach_actuator.call_count, len(self.motors_config_dict))

    def test_connect_failure_first_motor(self):
        self.mock_xyber_controller_instance.attach_actuator.return_value = False 
        motor_1_name = list(self.motors_config_dict.keys())[0]
        with self.assertRaisesRegex(RuntimeError, f"Failed to attach actuator '{motor_1_name}' for DCU '{self.config.dcu_name}'"):
            self.bus.connect()
        self.assertFalse(self.bus.is_connected) 
        motor_1_params = self.motors_config_dict[motor_1_name]
        self.mock_xyber_controller_instance.attach_actuator.assert_called_once_with(
            dcu_name=self.config.dcu_name,
            ch=to_dcu_channel_test_helper(motor_1_params[0]),
            type=to_dcu_actuator_type_test_helper(motor_1_params[2]),
            actuator_name=motor_1_name,
            can_id=motor_1_params[1]
        )
        self.mock_xyber_controller_instance.detach_actuator.assert_not_called() 

    def test_connect_failure_second_motor_detaches_first(self):
        attach_results = [True, False] 
        def side_effect_attach_actuator(*args, **kwargs): # pragma: no cover
            return attach_results.pop(0)
        self.mock_xyber_controller_instance.attach_actuator.side_effect = side_effect_attach_actuator
        self.mock_xyber_controller_instance.detach_actuator.return_value = True 

        motor_names_list = list(self.motors_config_dict.keys())
        motor_1_name = motor_names_list[0]
        motor_2_name = motor_names_list[1]

        with self.assertRaisesRegex(RuntimeError, f"Failed to attach actuator '{motor_2_name}' for DCU '{self.config.dcu_name}'"):
            self.bus.connect()

        self.assertFalse(self.bus.is_connected) 
        self.assertEqual(self.mock_xyber_controller_instance.attach_actuator.call_count, 2) 
        self.mock_xyber_controller_instance.detach_actuator.assert_called_once_with(
            self.config.dcu_name, motor_1_name
        )
    
    # 2. reconnect Method Tests
    @patch.object(AgibotX1MotorsBus, 'disconnect', autospec=True) 
    @patch.object(AgibotX1MotorsBus, 'connect', autospec=True)    
    def test_reconnect_when_not_connected(self, mock_bus_connect, mock_bus_disconnect):
        self.bus.is_connected = False 
        self.bus.reconnect()
        mock_bus_disconnect.assert_not_called() 
        self.mock_xyber_controller_instance.stop.assert_not_called() 
        mock_bus_connect.assert_called_once_with(self.bus) 

    @patch.object(AgibotX1MotorsBus, 'disconnect', autospec=True)
    @patch.object(AgibotX1MotorsBus, 'connect', autospec=True)
    def test_reconnect_when_connected(self, mock_bus_connect, mock_bus_disconnect):
        self.bus.is_connected = True 
        self.mock_xyber_controller_instance.stop.return_value = True 
        self.bus.reconnect()
        mock_bus_disconnect.assert_called_once_with(self.bus)
        mock_bus_connect.assert_called_once_with(self.bus)

    @patch.object(AgibotX1MotorsBus, 'disconnect', side_effect=RuntimeError("Disconnect failed test"), autospec=True)
    @patch.object(AgibotX1MotorsBus, 'connect', autospec=True)
    @patch('logging.error') 
    def test_reconnect_when_disconnect_fails_logs_error_and_continues(self, mock_log_error, mock_bus_connect, mock_bus_disconnect_method):
        self.bus.is_connected = True
        self.bus.reconnect()
        mock_bus_disconnect_method.assert_called_once_with(self.bus)
        mock_log_error.assert_called_once() 
        self.mock_xyber_controller_instance.stop.assert_called_once() 
        self.assertFalse(self.bus.is_connected) 
        mock_bus_connect.assert_called_once_with(self.bus) 

    # 3. apply_calibration Method Optimization Test
    def test_apply_calibration_motor_names_optimization(self):
        self.bus.calibration = {
            "motor_names": ["motor_1", "motor_2"], "calib_mode": ["DEGREE", "DEGREE"], 
            "drive_mode": [False, False], "homing_offset": [0.0, 0.0],
            "start_pos": [0.0,0.0], "end_pos": [1.0,1.0] 
        }
        self.bus.motor_names = MagicMock(return_value=list(self.motors_config_dict.keys()))
        test_values = np.array([10.0, 20.0], dtype=np.float32) 
        self.bus.apply_calibration(values=test_values, motor_names=["motor_1", "motor_2"])
        self.bus.motor_names.assert_not_called() 
        self.bus.motor_names.reset_mock() 
        self.bus.apply_calibration(values=test_values, motor_names=None)
        self.bus.motor_names.assert_called_once() 
    
    # 4. write Method Tests
    def _setup_for_write_tests(self, connect_bus=True):
        if connect_bus:
            self.bus.is_connected = True
        self.mock_xyber_controller_instance.set_mit_cmd.return_value = 0 # Default to success

    def test_write_not_connected(self):
        self.bus.is_connected = False
        with self.assertRaises(ConnectionError):
            self.bus.write(DATA_NAME_POSITION, [0.1, 0.2])

    def test_write_invalid_data_name(self):
        self._setup_for_write_tests()
        with self.assertRaisesRegex(ValueError, "Unknown data_name: invalid_data_name"):
            self.bus.write("invalid_data_name", [0.1, 0.2])

    def test_write_position_defaults(self):
        self._setup_for_write_tests()
        positions = [0.1, 0.2]
        self.bus.write(DATA_NAME_POSITION, positions)
        
        expected_calls = [
            call("motor_1", positions[0], MIT_CMD_DEFAULT_VEL, MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
            call("motor_2", positions[1], MIT_CMD_DEFAULT_VEL, MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

    def test_write_velocity_defaults(self):
        self._setup_for_write_tests()
        velocities = [0.5, -0.5]
        self.bus.write(DATA_NAME_VELOCITY, velocities)
        expected_calls = [
            call("motor_1", MIT_CMD_DEFAULT_POS, velocities[0], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
            call("motor_2", MIT_CMD_DEFAULT_POS, velocities[1], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

    def test_write_effort_defaults(self):
        self._setup_for_write_tests()
        efforts = [0.3, 0.4]
        self.bus.write(DATA_NAME_EFFORT, efforts)
        expected_calls = [
            call("motor_1", MIT_CMD_DEFAULT_POS, MIT_CMD_DEFAULT_VEL, efforts[0], self.bus.kp, self.bus.kd),
            call("motor_2", MIT_CMD_DEFAULT_POS, MIT_CMD_DEFAULT_VEL, efforts[1], self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

    def test_write_position_with_target_velocity_and_effort(self):
        self._setup_for_write_tests()
        positions = [0.1, 0.2]
        target_vels = [0.01, 0.02]
        target_effs = [0.001, 0.002]
        self.bus.write(DATA_NAME_POSITION, positions, target_vel=target_vels, target_eff=target_effs)
        expected_calls = [
            call("motor_1", positions[0], target_vels[0], target_effs[0], self.bus.kp, self.bus.kd),
            call("motor_2", positions[1], target_vels[1], target_effs[1], self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

    def test_write_optional_params_single_float_broadcast(self):
        self._setup_for_write_tests()
        positions = [0.1, 0.2]
        target_vel_single = 0.5
        self.bus.write(DATA_NAME_POSITION, positions, target_vel=target_vel_single)
        expected_calls = [
            call("motor_1", positions[0], target_vel_single, MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
            call("motor_2", positions[1], target_vel_single, MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

    def test_write_value_length_mismatch(self):
        self._setup_for_write_tests()
        with self.assertRaisesRegex(ValueError, "Length of values array must match"):
            self.bus.write(DATA_NAME_POSITION, [0.1]) # Only one value for two motors

    def test_write_optional_param_length_mismatch(self):
        self._setup_for_write_tests()
        with self.assertRaisesRegex(ValueError, "Length of optional parameter array must match"):
            self.bus.write(DATA_NAME_POSITION, [0.1, 0.2], target_vel=[0.5])


    @patch('logging.debug')
    def test_write_logging_debug_params(self, mock_log_debug):
        self._setup_for_write_tests()
        self.bus.write(DATA_NAME_POSITION, [0.1, 0.2])
        # Check that debug was called multiple times (for overall prep and per motor)
        self.assertGreaterEqual(mock_log_debug.call_count, 2 * len(self.motors_config_dict) + 1) # Rough check

    @patch('logging.info')
    def test_write_logging_info_on_success(self, mock_log_info):
        self._setup_for_write_tests()
        self.mock_xyber_controller_instance.set_mit_cmd.return_value = 0 # Success
        self.bus.write(DATA_NAME_POSITION, [0.1, 0.2])
        self.assertEqual(mock_log_info.call_count, len(self.motors_config_dict))
        for motor_name in self.motors_config_dict.keys():
            mock_log_info.assert_any_call(unittest.mock.string_containing(f"Motor '{motor_name}': Successfully set"))

    @patch('logging.warning')
    def test_write_logging_warning_on_failure(self, mock_log_warning):
        self._setup_for_write_tests()
        self.mock_xyber_controller_instance.set_mit_cmd.return_value = 1 # Failure
        self.bus.write(DATA_NAME_POSITION, [0.1, 0.2])
        self.assertEqual(mock_log_warning.call_count, len(self.motors_config_dict))
        for motor_name in self.motors_config_dict.keys():
            mock_log_warning.assert_any_call(unittest.mock.string_containing(f"Motor '{motor_name}': set_mit_cmd returned non-zero status: 1"))
            
    @patch.object(AgibotX1MotorsBus, 'revert_calibration', autospec=True)
    def test_write_revert_calibration_on_primary_only(self, mock_revert_calibration):
        self._setup_for_write_tests()
        # Define a simple calibration that revert_calibration can use
        self.bus.calibration = {
            "motor_names": ["motor_1", "motor_2"], "calib_mode": ["DEGREE", "DEGREE"],
            "drive_mode": [False, False], "homing_offset": [0.0, 0.0],
            "start_pos": [0.0,0.0], "end_pos": [1.0,1.0]
        }
        
        # Make revert_calibration return identifiable modified values
        def revert_side_effect(values, motor_names):
            return np.array(values) * 10 # e.g., scales input by 10
        mock_revert_calibration.side_effect = revert_side_effect

        positions = np.array([0.1, 0.2])
        target_vels = np.array([0.01, 0.02])
        
        # DATA_NAME_POSITION is in CALIBRATION_REQUIRED
        self.bus.write(DATA_NAME_POSITION, positions, target_vel=target_vels)
        
        # Check revert_calibration was called once with the primary position values
        mock_revert_calibration.assert_called_once()
        # Check the first arg of the first call to revert_calibration
        np.testing.assert_array_almost_equal(mock_revert_calibration.call_args[0][0], positions)

        # Check that set_mit_cmd received the reverted positions and original target_vels
        expected_reverted_positions = positions * 10
        expected_calls = [
            call("motor_1", expected_reverted_positions[0], target_vels[0], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
            call("motor_2", expected_reverted_positions[1], target_vels[1], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls)

        # Reset mocks for next test part
        mock_revert_calibration.reset_mock()
        self.mock_xyber_controller_instance.set_mit_cmd.reset_mock()

        # DATA_NAME_VELOCITY is NOT in CALIBRATION_REQUIRED
        self.bus.write(DATA_NAME_VELOCITY, target_vels, target_pos=positions)
        mock_revert_calibration.assert_not_called() # Should not be called for velocity

        expected_calls_vel = [
            call("motor_1", positions[0], target_vels[0], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
            call("motor_2", positions[1], target_vels[1], MIT_CMD_DEFAULT_EFF, self.bus.kp, self.bus.kd),
        ]
        self.mock_xyber_controller_instance.set_mit_cmd.assert_has_calls(expected_calls_vel)


if __name__ == '__main__':
    logging.basicConfig(level=logging.DEBUG) 
    unittest.main(verbosity=2)
