from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    ld = LaunchDescription()

    urdf_path = PathJoinSubstitution([
        FindPackageShare("evolutionary_explorer_ros"),         # Diretório do pacote `evolutionary_explorer_ros`
        "description",                   # Subpasta onde está o modelo
        "robot.urdf.xacro"               # Nome do arquivo Xacro
    ])

    # ParameterValue(..., value_type=str) e obrigatorio no Humble.
    robot_description_content = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str)

    robot_state_publisher_node = Node(package='robot_state_publisher',
                                      executable='robot_state_publisher',
                                      parameters=[{
                                          'robot_description': robot_description_content,
                                      }])

    ld.add_action(robot_state_publisher_node)


    ld.add_action(Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
    ))

    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("evolutionary_explorer_ros"),
        "rviz",
        "urdf.rviz"
    ])

    ld.add_action(Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
    ))

    return ld