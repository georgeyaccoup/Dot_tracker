from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(package='focus',
             executable='feedback', 
             name='feedback_node'
        ),
        Node(
            package='focus',
            executable='motor_x',
            name='motor_x_node',
            output='screen'
        ),
        Node(
            package='focus',
            executable='motor_y',
            name='motor_y_node',
            output='screen'
        ),
        Node(
            package='focus',
            executable='kinematics',
            name='kinematics_node',
            output='screen'
        ),
        Node(
            package='focus',
            executable='pid',
            name='pid_controller_node',
            output='screen'
        ),
        
        
        Node(
            package='focus',
            executable='color_detection',
            name='color_detection_node',
            output='screen'
        ),
    ])
