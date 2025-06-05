# teleoperate
```
sudo ~/X1/proj/bin/python lerobot/scripts/control_robot.py --robot.type=agibotx1 --control.type=teleoperate --control.fps=30
```

# record
```
sudo ~/X1/proj/bin/python lerobot/scripts/control_robot.py \
--robot.type=agibotx1 \
--control.type=record \
--control.fps 30 \
--control.single_task="Grasp a lego block and put it in the bin." \
--control.repo_id=lerobot/agidataset1 \
--control.num_episodes=1 \
--control.episode_time_s=10
```

# replay
```
sudo ~/X1/proj/bin/python lerobot/scripts/control_robot.py  \
--robot.type=agibotx1 \
--control.type=replay \
--control.fps=30 \
--control.repo_id=lerobot/agidataset1  \
--control.episode=0
```