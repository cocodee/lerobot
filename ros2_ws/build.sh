rm -rf install build log
colcon build --cmake-args -DPython3_EXECUTABLE=$(which python)
