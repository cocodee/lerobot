#!/bin/bash
conda activate ros2_env
sudo chmod 666 /dev/ttyTHS1
sudo chmod 666 /dev/ttyTHS2
#export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
export LD_PRELOAD=/usr/lib/aarch64-linux-gnu/libstdc++.so.6 python
export PYTHONPATH=/home/t/workspace/misumi_grippper/src/build:/home/t/workspace/eu_motor/build/lib:$PYTHONPATH