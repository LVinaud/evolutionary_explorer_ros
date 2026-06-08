from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, Command, LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessExit
from launch.conditions import IfCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


import os

# Comando para controlar o robô: ros2 run teleop_twist_keyboard teleop_twist_keyboard

def generate_launch_description():
    # Argumento para (des)ativar o RViz
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Se true, abre o RViz junto com o robo"
    )

    # Posicao/orientacao inicial do robo no mundo. Parametrizar o spawn e util
    # para testes e, na fase evolutiva, para variar o ponto de partida (robustez).
    spawn_x_arg = DeclareLaunchArgument("spawn_x", default_value="-8.0")
    spawn_y_arg = DeclareLaunchArgument("spawn_y", default_value="-0.5")
    # z=0.3: queda suave (a base do robo descansa em ~0.15 m). Alturas altas
    # (ex.: 1.0) causavam impacto forte no spawn e as vezes o robo CAPOTAVA.
    spawn_z_arg = DeclareLaunchArgument("spawn_z", default_value="0.3")
    spawn_yaw_arg = DeclareLaunchArgument("spawn_yaw", default_value="0.0")

    # ------------------------------------------------------
    # Caminho para o arquivo Xacro do robô
    # ------------------------------------------------------
    # Constrói o caminho absoluto para o arquivo `robot.urdf.xacro`,
    # localizado na pasta `description` do pacote `evolutionary_explorer_ros`.
    urdf_path = PathJoinSubstitution([
        FindPackageShare("evolutionary_explorer_ros"),         # Diretório do pacote `evolutionary_explorer_ros`
        "description",                   # Subpasta onde está o modelo
        "robot.urdf.xacro"               # Nome do arquivo Xacro
    ])

    # ------------------------------------------------------
    # Processamento do Xacro para gerar o URDF final
    # ------------------------------------------------------
    # Executa o comando `xacro <caminho>` em tempo de lançamento,
    # resultando no conteúdo URDF expandido como uma string.
    # ParameterValue(..., value_type=str) e obrigatorio no Humble: sem ele o
    # robot_description seria interpretado como YAML e falharia ("Unable to
    # parse the value of parameter robot_description as yaml").
    robot_urdf_final = ParameterValue(
        Command(["xacro ", urdf_path]), value_type=str)

    # ------------------------------------------------------
    # Nodo robot_state_publisher
    # ------------------------------------------------------
    # Publica as transformações dos links do robô com base no URDF.
    # Requer o parâmetro 'robot_description' com o conteúdo do modelo.
    diff_drive_params = PathJoinSubstitution([
        FindPackageShare("evolutionary_explorer_ros"),
        "config",
        "controller_config.yaml"
    ])
    
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": robot_urdf_final}
        ],
    )

    # ------------------------------------------------------
    # Preparação do sistema de controle das rodas do robô
    # ------------------------------------------------------

    # Inicialização do sistema de controle das juntas do robo
    # leitura do estado delas...
    load_joint_state_controller = ExecuteProcess(
        name="activate_joint_state_broadcaster",
        cmd=[
            "ros2",
            "control",
            "load_controller",
            "--set-state",
            "active",
            "joint_state_broadcaster",
        ],
        shell=False,
        output="screen",
    )

    # Inicialização do sistema de controle das rodas/motores do robo
    # o controle das rodas depende do estado das juntas
    start_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_diff_drive_base_controller",
        arguments=["diff_drive_base_controller"],
        parameters=[diff_drive_params],
        output="screen",
    )

    start_gripper_controller = Node(
        package="controller_manager",
        executable="spawner",
        name="spawner_gripper_controller",
        arguments=["gripper_controller"],
        parameters=[diff_drive_params],
        output="screen",
    )

    # Redireciona as mensagens do topico /diff_drive_base_controller/odom para /odom (Conveniencia)
    relay_odom = Node(
        name="relay_odom",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/diff_drive_base_controller/odom",
                "output_topic": "/odom",
            }
        ],
        output="screen",
    )

    # Redireciona as mensagens do topico /cmd_vel para /diff_drive_base_controller/cmd_vel_unstamped (Conveniencia)
    relay_cmd_vel = Node(
        name="relay_cmd_vel",
        package="topic_tools",
        executable="relay",
        parameters=[
            {
                "input_topic": "/cmd_vel",
                "output_topic": "/diff_drive_base_controller/cmd_vel_unstamped",
            }
        ],
        output="screen",
    )

    # ------------------------------------------------------
    # RViz: visualização do robô
    # ------------------------------------------------------
    # Carrega o arquivo de configuração do RViz a partir do pacote `evolutionary_explorer_ros`.
    rviz_config_file = PathJoinSubstitution([
        FindPackageShare("evolutionary_explorer_ros"),
        "rviz",
        "rviz_config.rviz"
    ])

    # use_rviz:=false desativa o RViz (util para testes headless e para os
    # episodios da computacao evolutiva, onde nao queremos GUI por individuo).
    rviz_node = Node(
        condition=IfCondition(LaunchConfiguration("use_rviz")),
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],  # Define o arquivo de configuração a ser carregado
    )

    # ------------------------------------------------------
    # Spawn do robô no simulador Gazebo
    # ------------------------------------------------------
    # Cria a entidade no mundo simulado utilizando a descrição do robô.
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        arguments=[
            "-name", "explorer_robot",          # Nome da entidade no simulador
            "-topic", "robot_description",      # Descrição do robô a ser utilizada
            "-x", LaunchConfiguration("spawn_x"),
            "-y", LaunchConfiguration("spawn_y"),
            "-z", LaunchConfiguration("spawn_z"),
            "-Y", LaunchConfiguration("spawn_yaw"),
            "--ros-args", "--log-level", "warn"
        ],
        parameters=[{"use_sim_time": True}],  # Usa o tempo simulado
    )
    
    # ------------------------------------------------------
    # Ponte Gazebo <-> ROS 2 (ros_gz_bridge)
    # ------------------------------------------------------
    # Estabelece a comunicação entre os tópicos do Gazebo e os tipos de mensagem do ROS 2.
    # Sintaxe do bridge: <topico no gazebo>@<tipo do gazebo>@<tipo do ros compativel>
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_explorer_robot",
        arguments=[
            "/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan",
            "/imu@sensor_msgs/msg/Imu@ignition.msgs.IMU",
            # Camera normal
            # "/robot_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Camera de segmentacao semantica
            "/robot_cam/labels_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/colored_map@sensor_msgs/msg/Image@ignition.msgs.Image",
            "/robot_cam/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",            
            # Camera de detectao bounding box
            # "/boxes_visible_2d_image@sensor_msgs/msg/Image@ignition.msgs.Image",
            # "/camera_info@sensor_msgs/msg/CameraInfo@ignition.msgs.CameraInfo",
            # Mensagem com anotacoes nao e suportado pelo ros_gz_bridge
            # Necessário para controladores como diff_drive_controller
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock",
            # Ground Truth de Posicao
            "/model/explorer_robot/pose@geometry_msgs/msg/Pose[ignition.msgs.Pose",
        ],
        output="screen",
    )

#  Nodo que publica odometria ground truth
    odom_gt= Node(
        package="evolutionary_explorer_ros",
        executable="ground_truth_odometry",
        name="odom_gt",
        arguments="",
        output="screen",
    )

#  Nodo que publica o mapa
    robo_mapper= Node(
        package="evolutionary_explorer_ros",
        executable="robo_mapper",
        name="robo_mapper",
        arguments="",
        output="screen",
    )

#  Casos vocês queiram carregar o controle do robô junto:
#  Não esquecer de descomentar a linha no LaunchDescription
#    controle= Node(
#        package="evolutionary_explorer_ros",
#        executable="controle_robo",
#        name="controle_do_robo",
#        arguments="",
#        output="screen",
#    )

    # ------------------------------------------------------
    # Definição da descrição completa do lançamento
    # ------------------------------------------------------
    # Inclui todos os nós definidos acima no lançamento.
    return LaunchDescription([
        use_rviz_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        bridge,
        robot_state_publisher_node,
        spawn_entity,
        RegisterEventHandler(
            event_handler=OnProcessExit(  
                target_action=spawn_entity,  # Após carregar o robo no simulador
                on_exit=[load_joint_state_controller], # Carrega o sistema de leitura das juntas
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=load_joint_state_controller, # Após carregar o sistema de leitura das juntas
                on_exit=[start_diff_controller], # Carrega o sistema de controle das rodas/motores
            )
        ),
        RegisterEventHandler(
            event_handler=OnProcessExit(
                target_action=start_diff_controller,
                on_exit=[start_gripper_controller],
            )
        ),        
        odom_gt,
        robo_mapper,
        rviz_node,
  #      relay_odom, # Nodos de redirecionamento de mensagens (Estamos usando apenas odom_gt agora)
        relay_cmd_vel # Nodos de redirecionamento de mensagens
  #      controle
    ])
