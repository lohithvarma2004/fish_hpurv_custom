import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'fish_hpurv'
    
    # Set the GAZEBO_PLUGIN_PATH to include UUV Gazebo ROS plugins
    set_gazebo_plugin_path = SetEnvironmentVariable(
        name='GAZEBO_PLUGIN_PATH',
        value=os.path.join(get_package_share_directory('uuv_gazebo_ros_plugins'), 'lib')
    )

    # Set the GAZEBO_MODEL_PATH to include both your fish model and the UUV world models.
    # This way, Gazebo can find your fish model as well as any models referenced by the UUV world.
    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(get_package_share_directory(package_name), 'models') + ':' +
              os.path.join(get_package_share_directory('uuv_gazebo_worlds'), 'models')
    )

    # Define file paths:
    # Path to your fish.sdf file (located under fish_hpurv/models/fish_hpurv/fish.sdf)
    sdf_file = os.path.join(get_package_share_directory(package_name), 'models', 'fish_hpurv', 'fish.sdf')
    # Path to the world file from uuv_gazebo_worlds package (ocean_waves.world)
    world_file_path = os.path.join(get_package_share_directory('uuv_gazebo_worlds'), 'worlds', 'ocean_waves.world')

    # Include the Gazebo launch file from gazebo_ros with the specified world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': world_file_path,
            'extra_gazebo_args': '--verbose -s libgazebo_ros_init.so -s libgazebo_ros_factory.so'
        }.items()
    )

    # Spawn the fish_hpurv model using its SDF file
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'fish_hpurv',
            '-file', sdf_file,
            '-x', '0', '-y', '0', '-z', '0'
        ],
        output='screen'
    )

    # Optional: Suppress ALSA warnings
    suppress_alsa_warnings = SetEnvironmentVariable(
        name='ALSA_CONFIG_PATH',
        value='/etc/alsa/alsa.conf'
    )

    ld = LaunchDescription()
    ld.add_action(set_gazebo_plugin_path)
    ld.add_action(set_gazebo_model_path)
    ld.add_action(suppress_alsa_warnings)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity_node)

    return ld