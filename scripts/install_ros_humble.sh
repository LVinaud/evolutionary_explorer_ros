#!/usr/bin/env bash
# ===========================================================================
# Instalacao do ROS 2 Humble + Gazebo Fortress (Ubuntu 22.04) e das
# dependencias do pacote evolutionary_explorer_ros.
#
# Uso:
#   chmod +x scripts/install_ros_humble.sh
#   ./scripts/install_ros_humble.sh            # perfil enxuto (ros-base + rviz2)
#   ROS_PROFILE=desktop ./scripts/install_ros_humble.sh   # perfil completo
#
# Requer ~6-8 GB livres no disco (o perfil 'desktop' usa mais).
# ===========================================================================
set -euo pipefail

ROS_PROFILE="${ROS_PROFILE:-base}"   # base | desktop

echo ">> [1/6] Configurando locale UTF-8"
sudo apt-get update
sudo apt-get install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo ">> [2/6] Habilitando o repositorio 'universe'"
sudo apt-get install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository -y universe

echo ">> [3/6] Adicionando a chave e o repositorio APT do ROS 2"
sudo mkdir -p /usr/share/keyrings
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

echo ">> [4/6] Atualizando indices APT"
sudo apt-get update

echo ">> [5/6] Instalando ROS 2 Humble (perfil: ${ROS_PROFILE}) + Gazebo Fortress"
if [ "${ROS_PROFILE}" = "desktop" ]; then
    ROS_CORE="ros-humble-desktop"
else
    # Perfil enxuto: ros-base + RViz (economiza disco)
    ROS_CORE="ros-humble-ros-base ros-humble-rviz2"
fi

sudo apt-get install -y \
    ${ROS_CORE} \
    ros-dev-tools \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-gz \
    ros-humble-ign-ros2-control \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-robot-state-publisher \
    ros-humble-joint-state-publisher \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-topic-tools \
    ros-humble-teleop-twist-keyboard \
    python3-opencv \
    python3-scipy \
    python3-numpy

# Libera espaco do cache de download
sudo apt-get clean

echo ">> [6/6] Inicializando o rosdep"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update

echo
echo ">> Concluido! Adicione o source do ROS ao seu ~/.bashrc:"
echo "   echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc"
echo
echo ">> Em seguida compile o workspace:"
echo "   cd ~/ros2_ws && colcon build --symlink-install --packages-select evolutionary_explorer_ros"
echo "   source install/setup.bash"
