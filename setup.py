from setuptools import find_packages, setup
from glob import glob
import os

# ---------------------------------------------------------------------------
# Compilando o workspace (a partir da raiz, ex.: ~/ros2_ws):
#   colcon build --symlink-install --packages-select evolutionary_explorer_ros
#   source install/setup.bash
# ---------------------------------------------------------------------------

package_name = 'evolutionary_explorer_ros'


def package_dir_tree(target_dir, base_install_path):
    """Coleta recursivamente todos os arquivos de target_dir e os mapeia para
    o caminho de instalacao correspondente sob base_install_path. Usado para
    instalar as pastas models/ e world/ preservando a sua estrutura."""
    entries = {}
    for filepath in glob(os.path.join(target_dir, '**'), recursive=True):
        if os.path.isfile(filepath):
            relpath = os.path.relpath(filepath, start=target_dir)
            install_path = os.path.join(base_install_path, os.path.dirname(relpath))
            entries.setdefault(install_path, []).append(filepath)
    return list(entries.items())


data_files = [
    # Requerido pelo ROS 2 (indice de pacotes e manifesto)
    ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
    ('share/' + package_name, ['package.xml']),

    # Recursos do nosso pacote
    (f'share/{package_name}/launch', glob('launch/*.py')),
    (f'share/{package_name}/description', glob('description/*.urdf.xacro')),
    (f'share/{package_name}/rviz', glob('rviz/*.rviz')),
    (f'share/{package_name}/config', glob('config/*.yaml')),
]

# Instala as pastas de modelos e mundos do Gazebo (se existirem)
if os.path.isdir('models'):
    data_files.extend(package_dir_tree('models', f'share/{package_name}/models'))
if os.path.isdir('world'):
    data_files.extend(package_dir_tree('world', f'share/{package_name}/world'))

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lazaro',
    maintainer_email='lazaropereiravn@gmail.com',
    description='SSC0712 Trabalho 1: exploracao, deteccao de bandeira e navegacao com maquina de estados (ROS 2 Humble + Gazebo Fortress).',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Cerebro da missao: maquina de estados + navegacao reativa
            'mission_control = evolutionary_explorer_ros.mission_control:main',
            # Percepcao: deteccao da bandeira via camera de segmentacao
            'flag_detector = evolutionary_explorer_ros.flag_detector:main',
            # Odometria ground-truth do simulador (TF odom -> base_link)
            'ground_truth_odometry = evolutionary_explorer_ros.ground_truth_odometry:main',
            # Mapeamento simples em grade de ocupacao (opcional)
            'robo_mapper = evolutionary_explorer_ros.robo_mapper:main',
        ],
    },
)
