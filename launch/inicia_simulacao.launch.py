from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution, FindExecutable, LaunchConfiguration
from launch.actions import SetEnvironmentVariable, ExecuteProcess, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node

import os

def generate_launch_description():
    # ------------------------------------------------------
    # Configuração de variáveis de ambiente para o Gazebo
    # ------------------------------------------------------
    # A variável GZ_SIM_SYSTEM_PLUGIN_PATH é usada para localizar plugins no Gazebo.
    # Ela é composta pelo caminho atual e pelo conteúdo de LD_LIBRARY_PATH.
    gz_env = {
        'GZ_SIM_SYSTEM_PLUGIN_PATH': ':'.join([
            os.environ.get('GZ_SIM_SYSTEM_PLUGIN_PATH', default=''),
            os.environ.get('LD_LIBRARY_PATH', default='')
        ]),
        # LIBGL_ALWAYS_SOFTWARE controla a renderizacao OpenGL. Em maquina
        # virtual sem aceleracao 3D (por exemplo VMware), a janela do Gazebo
        # fica em branco com a engine ogre2. Passe software_render:=1 para
        # renderizar por software, mais lento porem visivel.
        'LIBGL_ALWAYS_SOFTWARE': LaunchConfiguration('software_render'),
    }

    # Nível de verbosidade do Gazebo (0: silencioso, 4: mais detalhado)
    gz_verbosity = '3'

    # ------------------------------------------------------
    # Caminho para o mundo a ser carregado
    # ------------------------------------------------------

    # Verifica argumento com o nome do mundo que será simulado
    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='arena_cilindros.sdf',
        description='Nome do arquivo .sdf do mundo a ser carregado'
    )

    # Renderizacao por software (1) para VM sem aceleracao 3D, ou hardware (0).
    software_render_arg = DeclareLaunchArgument(
        'software_render',
        default_value='0',
        description='1 renderiza por software (VM sem GPU 3D), 0 usa a GPU'
    )

    # Encontra o diretório de instalação do pacote 'evolutionary_explorer_ros'.
    pkg_share = FindPackageShare("evolutionary_explorer_ros").find("evolutionary_explorer_ros")

    # Nome do arquivo do mundo (SDF) a ser carregado

    # Recupera dos parametros ou utiliza o default
    world_file_name =  world_file_name = LaunchConfiguration('world')

    # Caminho completo para o arquivo do mundo
    world_path = PathJoinSubstitution([
        pkg_share,
        "world",
        world_file_name
    ])

# Alguns teste utilizando cenários já existentes no gazebo
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/sensors_demo.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/heightmap.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/fuel.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/actor_crowd.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/auv_controls.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/buoyancy.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/fuel_textured_mesh.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/visualize_lidar.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/segmentation_camera.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/boundingbox_camera.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/spherical_coordinates.sdf'
#    world_path='/usr/share/ignition/ignition-gazebo6/worlds/rolling_shapes.sdf'

    # ------------------------------------------------------
    # Inicialização do simulador Gazebo
    # ------------------------------------------------------
    # Executa o comando: ign gazebo -r -v <verbosity> <world_path>
    # Inicia o Gazebo em modo headless (sem GUI), com nível de log definido.
    # Modo headless (sem GUI): util para testes automatizados e, futuramente,
    # para rodar episodios da computacao evolutiva sem interface grafica.
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Se true, executa o Gazebo apenas como servidor (-s, sem GUI)'
    )

    # GUI (padrao): inicia servidor + interface grafica.
    gazebo_gui = ExecuteProcess(
        condition=UnlessCondition(LaunchConfiguration('headless')),
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-r', '-v', gz_verbosity, world_path],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )

    # Headless: apenas o servidor (-s). Os sensores (camera/lidar) continuam
    # sendo renderizados pelo plugin Sensors do servidor.
    gazebo_headless = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('headless')),
        cmd=['ruby', FindExecutable(name="ign"), 'gazebo', '-s', '-r', '-v', gz_verbosity, world_path],
        output='screen',
        additional_env=gz_env,
        shell=False,
    )

    # ------------------------------------------------------
    # Configuração do caminho de recursos do Gazebo
    # ------------------------------------------------------
    # Define a variável de ambiente IGN_GAZEBO_RESOURCE_PATH para que o Gazebo
    # consiga localizar os modelos personalizados armazenados no pacote.
    gz_models_path = ":".join([
        pkg_share,
        os.path.join(pkg_share, "models")
    ])

    gz_set_env = SetEnvironmentVariable(
        name="IGN_GAZEBO_RESOURCE_PATH",
        value=gz_models_path,
    )

    # ------------------------------------------------------
    # Ponte Gazebo <-> ROS 2
    # ------------------------------------------------------
    # Estabelece comunicação entre a câmera do céu no Gazebo e o ROS 2.
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="ros_gz_bridge_world",
        arguments=[
            "/sky_cam@sensor_msgs/msg/Image@ignition.msgs.Image",
            # Necessário para controladores como diff_drive_controller
            "/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock"
        ],
        output="screen",
    )

    # ------------------------------------------------------
    # Descrição completa do lançamento
    # ------------------------------------------------------
    # Inclui as configurações de ambiente, a ponte e o lançamento do Gazebo.
    return LaunchDescription([
        world_file_arg,
        headless_arg,
        software_render_arg,
        gz_set_env,
        bridge,
        gazebo_gui,
        gazebo_headless
    ])
