#!/usr/bin/env python3
"""Launch da MISSAO COMPLETA (robo + sensores + controle autonomo).

Sobe tudo o que ja esta em carrega_robo.launch.py (robot_state_publisher,
spawn no Gazebo, ros2_control, bridge dos sensores, odometria ground-truth,
mapper e RViz) e adiciona os nodos de controle autonomo:

  * flag_detector   -> percepcao da bandeira (camera de segmentacao)
  * mission_control -> maquina de estados (exploracao/navegacao/posicionamento)

Uso (em 2 terminais, lembrando do `source install/setup.bash`):

  Terminal 1 (mundo):
    ros2 launch evolutionary_explorer_ros inicia_simulacao.launch.py

  Terminal 2 (robo + controle autonomo):
    ros2 launch evolutionary_explorer_ros missao.launch.py

Os ganhos de comportamento sao carregados de config/mission_params.yaml.
Para experimentar outros valores (inclusive vindos de um otimizador evolutivo),
passe outro arquivo:  `... missao.launch.py params_file:=/caminho/genoma.yaml`
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = FindPackageShare('evolutionary_explorer_ros')

    # Arquivo de parametros (o "genoma" da missao). Pode ser trocado em runtime.
    default_params = PathJoinSubstitution([
        pkg_share, 'config', 'mission_params.yaml'])
    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Arquivo YAML com os parametros de mission_control/flag_detector')
    params_file = LaunchConfiguration('params_file')

    # use_rviz pode ser repassado (ex.: use_rviz:=false para testes headless).
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='true',
        description='Abrir o RViz junto com a missao')
    use_rviz = LaunchConfiguration('use_rviz')

    # Posicao inicial do robo (repassada ao carrega_robo).
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='-8.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='-0.5')
    spawn_z_arg = DeclareLaunchArgument('spawn_z', default_value='0.3')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # Reaproveita o launch que carrega robo + sensores + bridge + rviz.
    carrega_robo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            pkg_share, 'launch', 'carrega_robo.launch.py'])),
        launch_arguments={
            'use_rviz': use_rviz,
            'spawn_x': LaunchConfiguration('spawn_x'),
            'spawn_y': LaunchConfiguration('spawn_y'),
            'spawn_z': LaunchConfiguration('spawn_z'),
            'spawn_yaw': LaunchConfiguration('spawn_yaw'),
        }.items(),
    )

    # Nodo de percepcao da bandeira.
    # use_sim_time=True: usa o relogio simulado (/clock) -> stamps coerentes.
    flag_detector = Node(
        package='evolutionary_explorer_ros',
        executable='flag_detector',
        name='flag_detector',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    # Nodo cerebro: maquina de estados da missao.
    # use_sim_time=True garante que serpentina/timeouts sigam o tempo da simulacao.
    mission_control = Node(
        package='evolutionary_explorer_ros',
        executable='mission_control',
        name='mission_control',
        output='screen',
        parameters=[params_file, {'use_sim_time': True}],
    )

    return LaunchDescription([
        params_file_arg,
        use_rviz_arg,
        spawn_x_arg,
        spawn_y_arg,
        spawn_z_arg,
        spawn_yaw_arg,
        carrega_robo,
        flag_detector,
        mission_control,
    ])
