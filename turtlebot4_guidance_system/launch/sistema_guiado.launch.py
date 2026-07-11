from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_turtlebot4_guidance_system = get_package_share_directory('turtlebot4_guidance_system')

    # Lanzar simulación + localización + Nav2
    sim_nav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_turtlebot4_guidance_system, 'launch', 'sim_turtlebot_nav.launch.py')
        )
    )

    # Detector YOLO
    yolo = Node(
        package='yolobot',
        executable='yolo_detector',
        name='yolo_detector',
        output='screen'
    )

    # Visualizador
    rqt = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        name='rqt_image_view',
        arguments=['/yolo/visualization'],
        output='screen'
    )

    # GUI del robot
    guia = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='turtlebot4_guidance_system',
                executable='robot_guia_gui.py',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        sim_nav,
        yolo,
        rqt,
        guia
    ])
