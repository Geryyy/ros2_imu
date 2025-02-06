from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from datetime import datetime
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():

    imu_node = Node(
                package='imu',
                executable='imu_talker',
                name='imu_talker',
                output='both'
            )

    return LaunchDescription([
        imu_node
    ])
