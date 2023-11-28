import launch
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ros2_tcp_interface_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("tcp_interface") + "/launch/tcp_interface.launch.py"]))
    rr_manual_node_launch = launch.actions.IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("manual_node") + "/launch/manual_node.launch.py"]))
    return launch.LaunchDescription([
        ros2_tcp_interface_launch,
        rr_manual_node_launch
    ])