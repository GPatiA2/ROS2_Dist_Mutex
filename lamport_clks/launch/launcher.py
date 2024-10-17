from launch import LaunchDescription, LaunchContext
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, TextSubstitution

def launch_nodes(context : LaunchContext):
    num_node = LaunchConfiguration('num_id').perform(context)
    return [
        Node(
            package='lamport_clks',
            namespace='node_'+num_node,
            executable='sample_node',
            name='node_'+num_node,
            parameters=[{
               'num_id' :LaunchConfiguration('num_id') 
            }]
        )
    ]

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'num_id',
            default_value='1',
            description='The id of the node'
        ),
        OpaqueFunction(function=launch_nodes) 
    ])