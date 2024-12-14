docker compose up -d

docker exec -it zed bash

source devel/setup.bash
export ROS_IP=192.168.31.7
export ROS_MASTER_URI=http://192.168.31.105:11311

roslaunch zed_wrapper zed2_robot.launch