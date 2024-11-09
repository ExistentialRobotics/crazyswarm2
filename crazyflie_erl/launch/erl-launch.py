import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    model_params = os.path.join(
        get_package_share_directory('crazyflie_erl'),
        'config',
        'crazyflie_model_params.yaml'
        )
    
    cf_param = os.path.join(
        get_package_share_directory('crazyflie_erl'),
        'config',
        'crazyflies_param.yaml'
        )

    yo_lqr_param = os.path.join(
        get_package_share_directory('crazyflie_erl'),
        'config',
        'yo_lqr_params.yaml'
        )
    
    node=Node(
        package = 'crazyflie_erl',
        name = 'CS2ERLCommander_node',
        executable = 'YO_LQR_node',
        parameters = [model_params, cf_param, yo_lqr_param]
    )

    ld.add_action(node)
    return ld