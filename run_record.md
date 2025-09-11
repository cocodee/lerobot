## 1. 启动ros2 controller
cd ~/workspace/supre_robot_control
./start_common_leader_follower.sh
## 2. 运行lerobot脚本
cd ~/workspace/gitprj/lerobot-env/lerobot
运行run_teleop.sh,lerobot命令可以自行更改

python -m lerobot.record \
    --robot.type=ros2_dual_follower \
    --robot.id=eyou_follower \
    --teleop.type=ros2_dual_leader \
    --teleop.id=eyou_leader \
    --robot.cameras="{head_cam: {type: opencv, index_or_path: 0, width: 640, height: 480, fps: 30}, right_wrist_cam: {type: opencv, index_or_path: 2, width: 640, height: 480, fps: 30}, left_wrist_cam: {type: opencv, index_or_path: 4, width: 640, height: 480, fps: 30}}" \
    --dataset.single_task="Grasp the workpiece and put it in the appropriate position." \
    --dataset.repo_id=supdata/dataset_0828_1 \
    --dataset.episode_time_s=150 \
    --dataset.num_episodes=10 \
    --dataset.reset_time_s=10