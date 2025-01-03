import os

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # load crazyflies
    # crazyflies_yaml = os.path.join(
    #     get_package_share_directory('crazyflie'),
    #     'config',
    #     'crazyflies.yaml')

    # with open(crazyflies_yaml, 'r') as ymlfile:
    #     crazyflies = yaml.safe_load(ymlfile)

    # server_params = crazyflies

    # cf_examples_dir = get_package_share_directory('crazyflie_examples')
    # bringup_dir = get_package_share_directory('nav2_bringup')
    # bringup_launch_dir = os.path.join(bringup_dir, 'launch')

    return LaunchDescription([
        Node(package='crazyflie_examples',
            namespace='tro_experiment',
            exectuable='data_logger.py',
            name='data_logger',
            output='screen'
        ),
        Node(package='crazyflie_examples',
            namespace='tro_experiment',
            exectuable='test_nn_network.py',
            name='experiment',
            output='screen'
        )
    ])
