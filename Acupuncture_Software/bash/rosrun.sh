#!/bin/bash
gnome-terminal -- bash -c "exec bash"
# gnome-terminal --window -e 'bash -c "cd /home/ros/AcuRobot; catkin_make; source ./devel/setup.bash; rosrun mainfunc mainfunc; exec bash"' \
# --tab -e 'bash -c "sleep 2s; cd /home/ros/AcuRobot; source ./devel/setup.bash; rosrun start_force_sensing start_force_sensing; exec bash"' \
# --tab -e 'bash -c "sleep 2s; cd /home/ros/AcuRobot; source ./devel/setup.bash; rosrun joint_reset joint_reset; exec bash"' \
# --tab -e 'bash -c "sleep 2s; cd /home/ros/AcuRobot; source ./devel/setup.bash; rosrun start_rough_positioning start_rough_positioning; exec bash"' \
# --tab -e 'bash -c "sleep 2s; cd /home/ros/AcuRobot; source ./devel/setup.bash; rosrun start_accurate_positioning_1 start_accurate_positioning_1; exec bash"' \

# gnome-terminal --window -e 'bash -c "rostopic list; exec bash"' \
# --tab -e 'bash -c "sleep 3s; rosnode list; exec bash"' \
# --tab -e 'bash -c "sleep 5s; rosnode list; exec bash"' \

# sleep后的参数是整个窗口从启动开始所经过的时间

