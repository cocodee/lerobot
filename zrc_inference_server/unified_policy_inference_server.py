import time
import logging
import traceback
import json 
import zenoh
from typing import Dict, Any, Optional
from dataclasses import dataclass, field

from zrc.core import ZRCNode
from zrc.action import ActionServer, ActionHandle, ActionStatus
from zrc.exceptions import ZRCError

# --- LeRobot Imports ---
from lerobot.configs import parser
from lerobot.robots import RobotConfig

# --- Local Imports ---
from .inference_config import LocalInferenceConfig, RemoteInferenceConfig
from .inference_engine import InferenceEngine, LocalInferenceEngine, RemoteInferenceEngine

# --- Global Setup ---
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('UnifiedInferenceServer')
ACTION_NAME = "policy_inference"


@dataclass
class UnifiedServerConfig:
    """Unified configuration for the inference server."""
    inference_mode: str = "local"  # "local" or "remote"

    # Common configurations
    robot: RobotConfig | None = None
    zenoh_config: Dict[str, Any] = field(default_factory=dict) 
    node_name: str = "lerobot_unified_server"
    control_freq: int = 30

    # Mode-specific configurations
    local: Optional[LocalInferenceConfig] = field(default=None)
    remote: Optional[RemoteInferenceConfig] = field(default=None)

    def __post_init__(self):
        if self.inference_mode == "local" and self.local is None:
            raise ValueError("Inference mode is 'local' but 'local' config is missing.")
        if self.inference_mode == "remote" and self.remote is None:
            raise ValueError("Inference mode is 'remote' but 'remote' config is missing.")
        if self.inference_mode not in ["local", "remote"]:
            raise ValueError(f"Invalid inference_mode: {self.inference_mode}. Must be 'local' or 'remote'.")

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        # Allows --local.policy=/path/to/policy from CLI
        return ["local.policy"]




# #######################################################
#  SECTION 4: UNIFIED POLICY INFERENCE SERVER
# #######################################################

class UnifiedPolicyInferenceServer:
    """A unified ZRC Action Server for policy inference."""
    def __init__(self, node: ZRCNode, config: UnifiedServerConfig):
        self.node = node
        self.config = config
        logger.info(f"Unified Policy Inference Server starting in '{config.inference_mode}' mode.")
        
        self._action_server = ActionServer(
            self.node, 
            ACTION_NAME,
            execute_callback=self.execute_callback,
            data_serializer='json'
        )
        logger.info(f"ZRC Action Server '{ACTION_NAME}' is ready.")

    def execute_callback(self, goal_id: str, goal_data: Dict[str, Any], handle: ActionHandle):
        task_description = goal_data.get('task_description')
        num_inference_steps = goal_data.get('num_inference_steps', 0)

        logger.info(f"Executing goal {goal_id[:8]}... (task='{task_description}', steps={num_inference_steps})")
        
        engine: Optional[InferenceEngine] = None
        try:
            # --- Engine Factory ---
            if self.config.inference_mode == "local":
                engine = LocalInferenceEngine(self.config.local, self.config.robot, self.config.control_freq)
            elif self.config.inference_mode == "remote":
                engine = RemoteInferenceEngine(self.config.remote, self.config.robot, self.config.control_freq)
            else:
                # This case is already checked in config, but good for safety
                raise NotImplementedError(f"Inference mode '{self.config.inference_mode}' not supported.")
            
            # --- Generic Execution Loop ---
            engine.setup(task_description)
            
            sleep_time = 1.0 / self.config.control_freq
            for i in range(num_inference_steps):
                if handle.is_cancel_requested():
                    logger.info(f'Goal {goal_id[:8]}... canceled by client.')
                    handle.publish_result({"success": False, "message": "Goal was canceled."}, ActionStatus.PREEMPTED)
                    return

                loop_start_time = time.perf_counter()
                engine.step()
                
                handle.publish_feedback({"current_step": i + 1, "status": f"Running step {i+1}/{num_inference_steps}"})
                
                elapsed = time.perf_counter() - loop_start_time
                time.sleep(max(0, sleep_time - elapsed))

            handle.publish_result({"success": True, "message": "Inference completed successfully."}, ActionStatus.SUCCEEDED)

        except Exception as e:
            logger.error(f"Error during execution of {goal_id[:8]}...: {e}")
            logger.error(traceback.format_exc())
            handle.publish_result({"success": False, "message": f"Execution failed: {e}"}, ActionStatus.ABORTED)
        
        finally:
            if engine:
                engine.cleanup()
            logger.info(f"Execution finished for goal {goal_id[:8]}...")


# #######################################################
#  SECTION 5: MAIN ENTRYPOINT
# #######################################################

@parser.wrap_for_main()
def main(config: UnifiedServerConfig):
    try:
        if not config.zenoh_config:
            zrc_config_to_pass = zenoh.Config()
        else:
            zenoh_conf = zenoh.Config()
            zenoh_conf.from_json5(json.dumps(config.zenoh_config))
            zrc_config_to_pass = zenoh_conf
        zrc_node = ZRCNode(config.node_name, config=zrc_config_to_pass)
        logger.info(f"ZRC Node '{config.node_name}' initialized.")
    except ZRCError as e:
        logger.fatal(f"Failed to initialize ZRC Node: {e}")
        return

    try:
        server = UnifiedPolicyInferenceServer(node=zrc_node, config=config)
    except Exception as e:
        logger.exception(f"Server instantiation failed: {e}")
        zrc_node.close()
        return
        
    try:
        logger.info(f"Server is running. Press Ctrl+C to stop.")
        while True:
            time.sleep(0.1)
    except KeyboardInterrupt:
        logger.info("Server shutting down.")
    finally:
        zrc_node.close()
        logger.info("ZRC Session closed.")

if __name__ == '__main__':
    # Make sure to paste the full RobotClient class implementation above
    # where the placeholder `pass` is, otherwise this will fail.
    # Note: `parser.wrap_for_main()` now automatically handles CLI parsing
    # and calls the main function.
    main()