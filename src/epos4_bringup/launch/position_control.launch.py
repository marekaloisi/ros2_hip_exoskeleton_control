from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node

def generate_launch_description():
    left_node = Node(
        package='epos4_node_position_controller_left',
        executable='epos4_controller_left',
        name='left_controller',
        output='screen'
    )

    right_node = Node(
        package='epos4_node_position_controller_right',
        executable='epos4_controller_right',
        name='right_controller',
        output='screen'
    )

 
    gait_manager_node = TimerAction(
        period=3.0,
        actions= [Node(
            package='epos4_gait_manager',
            executable='epos4_gait_manager',
            name='gait_manager',
            output='screen'
        )]
    )


    gait_preidction_node = TimerAction(
        period=1.0,
        actions= [Node(
            package='epos4_gait_parameter_prediction_node',
            executable='epos4_gait_parameter_prediction_node',
            name='gait_prediction',
            output='screen'
        )]
    )


    return LaunchDescription([
        left_node,
        right_node,
        gait_manager_node,
        gait_preidction_node
    ])
