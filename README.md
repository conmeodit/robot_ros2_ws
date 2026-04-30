#code+pull
cd ~/robot_ros2_ws
git pull origin main 
cd ~/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
source ~/robot_ros2_ws/install/setup.bash
cd /home/linhpham/robot_ros2_ws 
./upload_mega.sh
#build
cd ~/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
#automap
cd ~/robot_ros2_ws
git pull origin main 
cd ~/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch vacuum_driver autonomous_mapping.launch.py
#rviz2
cd ~/robot_ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
rviz2 -d src/vacuum_driver/rviz/mapping.rviz


ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

sudo nmcli device wifi rescan
nmcli device wifi list
sudo nmcli device wifi connect "tk" password "mk"
sudo nmcli connection delete "TÊN_WIFI_CŨ"