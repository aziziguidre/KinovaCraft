import os
import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file reserve pour la simulation MATLAB :
      1. robot_state_publisher (charge l'URDF Kinova Gen3 6-DOF)
      2. rviz2 (affiche le robot en 3D)
      -> Aucun analyzer_node ni joint_state_publisher Python.
      -> Le robot attend directement les messages de MATLAB sur /joint_states.
    """
    pkg_description_share = get_package_share_directory('kortex_description')
    pkg_planner_share = get_package_share_directory('trajectory_planner_g6')

    # Conversion Xacro -> URDF Kinova Gen3 6-DOF avec pince Robotiq 2F-85
    xacro_file = os.path.join(
        pkg_description_share,
        'robots',
        'kinova.urdf.xacro'
    )

    if os.path.exists(xacro_file):
        try:
            doc = xacro.process_file(
                xacro_file,
                mappings={
                    'name': 'gen3',
                    'arm': 'gen3',
                    'dof': '6',
                    'gripper': 'robotiq_2f_85'
                }
            )
            robot_description_content = doc.toxml()
        except Exception:
            urdf_static = os.path.join(
                pkg_description_share,
                'robots',
                'gen3_6dof_robotiq_2f_85.urdf'
            )
            with open(urdf_static, 'r', encoding='utf-8') as f:
                robot_description_content = f.read()
    else:
        urdf_static = os.path.join(
            pkg_description_share,
            'robots',
            'gen3_6dof_robotiq_2f_85.urdf'
        )
        with open(urdf_static, 'r', encoding='utf-8') as f:
            robot_description_content = f.read()

    rviz_config_path = os.path.join(
        pkg_planner_share,
        'rviz',
        'trajectory_viz.rviz'
    )

    # 1. robot_state_publisher (attend /joint_states)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}]
    )

    # 2. RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        rviz_node
    ])
