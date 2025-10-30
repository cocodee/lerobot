import time
import json
import threading
import logging

import zenoh
import zrc
from zrc.action import ActionClient, ActionStatus, GoalHandle

# --- 配置 ---
# 确保这个配置与你的 Zenoh 网络匹配
# 如果服务器和客户端在同一台机器上，这个默认配置通常可以工作
ZENOH_CONFIG = {"mode": "peer"} 

# 服务器的 Action 名称
ACTION_NAME = "policy_inference" 

# 客户端节点名称 (最好是唯一的)
CLIENT_NODE_NAME = f"test_client_{int(time.time())}"

# 日志设置
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('TestClient')

# --- 用于同步异步回调的全局变量 ---
# 使用一个字典来存储不同测试的结果和事件
test_context = {
    "result": None,
    "feedback_list": [],
    "result_received_event": threading.Event()
}

# --- 回调函数 ---
def on_result(result: dict):
    """当 Action 完成、失败或被取消时调用的回调函数"""
    status = result.get('status', 'UNKNOWN')
    data = result.get('data', {})
    logger.info(f"🏁 最终结果已收到! 状态: {status}, 数据: {data}")
    test_context["result"] = result
    test_context["result_received_event"].set() # 发送信号，通知主线程结果已收到

def on_feedback(feedback: dict):
    """在 Action 执行期间接收反馈的回调函数"""
    data = feedback.get('data', {})
    logger.info(f"  [反馈] -> {data}")
    test_context["feedback_list"].append(data)

def reset_test_context():
    """在每个测试用例开始前重置上下文"""
    test_context["result"] = None
    test_context["feedback_list"].clear()
    test_context["result_received_event"].clear()


def main():
    """主测试函数"""
    logger.info("初始化 ZRC 节点...")
    
    # 配置并创建 ZRC 节点
    config = zenoh.Config()
    config.from_json5(json.dumps(ZENOH_CONFIG))
    node = zrc.ZRCNode(CLIENT_NODE_NAME, config=config)

    try:
        logger.info(f"节点 '{CLIENT_NODE_NAME}' 已启动. 创建 Action 客户端...")
        # 创建一个 ActionClient 来与服务器通信
        action_client = ActionClient(
            node, 
            ACTION_NAME, 
            data_serializer='json'
        )

        # 等待客户端发现服务器（在网络繁忙时可能需要）
        time.sleep(1) 
        
        # =================================================================
        #  测试场景 1: 成功执行
        # =================================================================
        print("\n" + "="*50)
        logger.info("🚀 开始测试场景 1: 成功执行")
        print("="*50)
        reset_test_context()

        goal_data_success = {
            "task_description": "Grasp the workpiece and put it in the appropriate position.",
            "num_inference_steps": 40
        }
        
        logger.info(f"发送目标: {goal_data_success}")
        action_client.send_goal(
            goal_data=goal_data_success,
            result_callback=on_result,
            feedback_callback=on_feedback
        )
        
        # 等待结果，设置5秒超时
        logger.info("... 等待任务完成 ...")
        completed_in_time = test_context["result_received_event"].wait(timeout=5.0)

        if not completed_in_time:
            logger.error("❌ 测试失败: 等待结果超时!")
        else:
            final_status = test_context["result"].get("status")
            if final_status == ActionStatus.SUCCEEDED:
                logger.info("✅ 测试成功: 任务按预期完成!")
            else:
                logger.error(f"❌ 测试失败: 任务状态为 {final_status}, 而不是 SUCCEEDED.")
        
        print("\n" + "-"*20 + " 场景 1 结束 " + "-"*20 + "\n")
        time.sleep(2) # 在测试之间留出间隔


        # =================================================================
        #  测试场景 2: 任务取消
        # =================================================================
        print("\n" + "="*50)
        logger.info("🚀 开始测试场景 2: 任务中途取消")
        print("="*50)
        reset_test_context()

        goal_data_cancel = {
            "task_description": "Grasp the workpiece and put it in the appropriate position.",
            "num_inference_steps": 100 # 设置一个较长的步骤数，确保我们有时间取消
        }

        logger.info(f"发送一个长任务目标: {goal_data_cancel}")
        goal_handle: GoalHandle = action_client.send_goal(
            goal_data=goal_data_cancel,
            result_callback=on_result,
            feedback_callback=on_feedback
        )

        # 等待一小段时间，让任务开始执行
        logger.info("... 任务已发送, 等待 0.2 秒后发送取消请求 ...")
        time.sleep(0.2)
        
        logger.info("🛑 发送取消请求!")
        goal_handle.cancel()

        # 等待取消后的结果
        logger.info("... 等待取消确认结果 ...")
        completed_in_time = test_context["result_received_event"].wait(timeout=5.0)

        if not completed_in_time:
            logger.error("❌ 测试失败: 等待取消结果超时!")
        else:
            final_status = test_context["result"].get("status")
            if final_status == ActionStatus.PREEMPTED:
                logger.info("✅ 测试成功: 任务被成功取消 (状态 PREEMPTED)!")
                logger.info(f"任务在被取消前执行了 {len(test_context['feedback_list'])} 步。")
            else:
                logger.error(f"❌ 测试失败: 任务状态为 {final_status}, 而不是 PREEMPTED.")

        print("\n" + "-"*20 + " 场景 2 结束 " + "-"*20 + "\n")

    except Exception as e:
        logger.error(f"客户端执行时发生错误: {e}")
    finally:
        logger.info("关闭 ZRC 节点...")
        node.close()
        logger.info("客户端已关闭。")

if __name__ == "__main__":
    main()