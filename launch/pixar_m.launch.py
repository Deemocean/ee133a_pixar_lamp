"""Launch the momentum test

This should start
  1) RVIZ, ready to view the robot
  2) The robot_state_publisher to broadcast the robot model
  3) The HW7 P3 code to move the joints

"""

import os
import xacro

from ament_index_python.packages import get_package_share_directory as pkgdir

from launch                            import LaunchDescription
from launch.actions                    import DeclareLaunchArgument
from launch.actions                    import OpaqueFunction
from launch.actions                    import Shutdown
from launch.substitutions              import LaunchConfiguration
from launch_ros.actions                import Node


#
# Generate the Launch Description
#
def generate_launch_description():

    ######################################################################
    # LOCATE FILES

    # Locate the RVIZ configuration file.
    rvizcfg = os.path.join(pkgdir('pixar'), 'rviz/pixar.rviz')

    # Locate the URDF file
    lamp_urdf = os.path.join(pkgdir('pixar'), 'urdf/lamp.urdf')
    scene_urdf = os.path.join(pkgdir('pixar'), 'urdf/scene.urdf')

    # Load the robot's URDF file (XML).
    with open(lamp_urdf, 'r') as file:
        lamp_description = file.read()
        print(lamp_description)

    with open(scene_urdf, 'r') as file:
        scene_description = file.read()

    ######################################################################
    # PREPARE THE LAUNCH ELEMENTS

    # Configure a node for the robot_state_publisher.
    # node_tf = Node(
    #     name       = 'tf', 
    #     package    = 'pixar',
    #     executable = 'broadcaster',
    #     output     = 'screen')

    # Configure a node for the robot_state_publisher.
    node_robot_state_publisher_lamp = Node(
        name       = 'robot_state_publisher', 
        namespace="lamp",
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': lamp_description}])
    
    node_robot_state_publisher_scene = Node(
        name       = 'robot_state_publisher', 
        namespace="scene",
        package    = 'robot_state_publisher',
        executable = 'robot_state_publisher',
        output     = 'screen',
        parameters = [{'robot_description': scene_description}])
    # Configure a node for RVIZ
    node_rviz = Node(
        name       = 'rviz', 
        package    = 'rviz2',
        executable = 'rviz2',
        output     = 'screen',
        arguments  = ['-d', rvizcfg],
        on_exit    = Shutdown())
    
    node_marker = Node(
        name       = 'marker', 
        package    = 'pixar',
        executable = 'marker',
        output     = 'screen')

    node_trajectory = Node(
        name       = 'trajectory', 
        package    = 'pixar',
        executable = 'momentum',
        output     = 'screen')


    ######################################################################
    # RETURN THE ELEMENTS IN ONE LIST
    return LaunchDescription([
        # Start the robot_state_publisher, RVIZ, and the trajectory.
        node_robot_state_publisher_lamp,
        node_robot_state_publisher_scene,
        node_rviz,
        node_trajectory,
        node_marker,
    ])
