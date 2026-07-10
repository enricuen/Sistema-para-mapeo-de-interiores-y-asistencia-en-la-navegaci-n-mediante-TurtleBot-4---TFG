# These are some of the commands that were used during the tutorial. This file should not be run, it is to be used as a reference.
source /opt/ros/jazzy/setup.bash
source ~/.bashrc
#terminal 1
ros2 run rmw_zenoh_cpp rmw_zenohd
#slam
ros2 launch dis_tutorial3 sim_turtlebot_slam.launch.py world:=prueba_1
#navegacion
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py
ros2 launch dis_tutorial3 sim_turtlebot_nav.launch.py x:=-9 y:=-2 z:=0.0 yaw:=-3.1415
#deteccion señal
ros2 run yolobot yolo_detector
#visualizador deteccion
ros2 run rqt_image_view rqt_image_view 
#navegador 
ros2 run dis_tutorial3 robot_guia.py 
#matar processo fantasma
killall -9 gzserver gzclient rviz2 ruby ; pkill -9 -f ros2 ; pkill -9 -f gazebo
ros2 daemon stop && ros2 daemon start
#para habilitar P2P
export ZENOH_ROUTER_CHECK_ATTEMPTS=-1
export ZENOH_CONFIG_OVERRIDE='scouting/multicast/enabled=true'


ros2 launch turtlebot4_guidance_system sistema_guiado.launch.py 

