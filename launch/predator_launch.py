from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. Start the actual simulation
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='sim'
        ),
        
        # 2. Spawn the predator turtle
        Node(
            package='predator_turtle',
            executable='spawner',
            name='spawner'
        ),

        # 3. Broadcast where 'turtle1' (the prey) is
        Node(
            package='predator_turtle',
            executable='tf_broadcaster',
            name='broadcaster1',
            parameters=[
                {'turtlename': 'turtle1'}
            ]
        ),

        # 4. Broadcast where 'predator' is
        Node(
            package='predator_turtle',
            executable='tf_broadcaster',
            name='broadcaster2',
            parameters=[
                {'turtlename': 'predator'}
            ]
        ),

        # 5. Start the listener (The Brain) to control the predator
        Node(
            package='predator_turtle',
            executable='listener',
            name='listener'
        ),
    ])