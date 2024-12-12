#!/bin/zsh
echo "------------------------[Cleaning]------------------------"
rm -r ./build ./install ./log

echo "------------------------[Building]------------------------"
colcon build

echo "------------------------[Launching]------------------------"
ros2 launch pixar pixar_m.launch.py