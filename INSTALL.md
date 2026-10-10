```markdown
# Instalação do Ambiente ROS 2 - FBOT (Bóris)
Este guia contém o passo a passo completo para configurar uma máquina nova com Ubuntu 22.04 e ROS 2 Humble, clonar os pacotes da equipe e rodar a simulação do Bóris no Gazebo Ignition Fortress.

## Parte 1: Ubuntu 22.04, ROS 2 Humble e Dependências

### 1.1 Configurar o Repositório Oficial do ROS 2 Humble
```bash
sudo apt update && sudo apt install -y software-properties-common curl gnupg lsb-release
sudo add-apt-repository universe -y
sudo apt update

# Instala o pacote oficial de chaves e repositório do ROS 2
export ROS_APT_SOURCE_VERSION=$(curl -s [https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest](https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest) | grep -F "tag_name" | awk -F\" '{print $4}')
curl -L -o /tmp/ros2-apt-source.deb "[https://github.com/ros-infrastructure/ros-apt-source/releases/download/$](https://github.com/ros-infrastructure/ros-apt-source/releases/download/$){ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt update && sudo apt upgrade -y

```

### 1.2 Instalar o ROS 2 Desktop e Ferramentas de Compilação

```bash
sudo apt install -y \
ros-humble-desktop \
ros-dev-tools \
python3-pip \
python3-colcon-common-extensions \
python3-rosdep \
python3-vcstool \
git \
redis-server

```

### 1.3 Instalar o Simulador Gazebo e Pacotes do Bóris

Este comando único instala todos os pacotes do ecossistema ROS 2 da equipe, incluindo Gazebo, Nav2, MoveIt e localização.

```bash
sudo apt install -y \
ros-humble-ros-gz \
ros-humble-ros-gz-bridge \
ros-humble-ros-gz-sim \
ros-humble-ros-gz-image \
ros-humble-ros-gz-interfaces \
ros-humble-gz-ros2-control \
ros-humble-ros2-controllers \
ros-humble-controller-manager \
ros-humble-diff-drive-controller \
ros-humble-joint-state-broadcaster \
ros-humble-joint-trajectory-controller \
ros-humble-gripper-controllers \
ros-humble-rqt-joint-trajectory-controller \
ros-humble-joint-state-publisher-gui \
ros-humble-xacro \
ros-humble-navigation2 \
ros-humble-nav2-bringup \
ros-humble-slam-toolbox \
ros-humble-teleop-twist-keyboard \
ros-humble-teleop-twist-joy \
ros-humble-topic-tools \
ros-humble-tf-transformations \
ros-humble-moveit \
ros-humble-moveit-planners-chomp \
ros-humble-pilz-industrial-motion-planner \
ros-humble-moveit-task-constructor-core \
ros-humble-moveit-task-constructor-capabilities \
ros-humble-moveit-task-constructor-msgs \
ros-humble-yasmin \
ros-humble-yasmin-ros \
ros-humble-bno055 \
ros-humble-sick-scan-xd \
ros-humble-urg-node \
ros-humble-velodyne-description \
ros-humble-depthimage-to-laserscan \
ros-humble-pcl-conversions \
ros-humble-pcl-msgs \
ros-humble-audio-common-msgs \
ros-humble-vision-msgs \
ros-humble-object-recognition-msgs \
ros-humble-rosbridge-server \
ros-humble-async-web-server-cpp \
ros-humble-web-video-server \
ros-humble-robot-localization \
ros-humble-rmw-cyclonedds-cpp

```

## Parte 2: Criação do Workspace e Clonagem dos Repositórios

### 2.1 Criar a Estrutura do fbot_ws

```bash
mkdir -p ~/fbot_ws/src
cd ~/fbot_ws/src

```

### 2.2 Clonar Repositórios

**Nota:** Verifique se sua chave SSH está cadastrada. Caso use token, substitua `git@github.com:` por `https://github.com/`.

```bash
cd ~/fbot_ws/src

# Repositórios principais (branch main)
git clone -b main [https://github.com/fbotathome/fbot_bringup.git](https://github.com/fbotathome/fbot_bringup.git)
git clone -b main [https://github.com/fbotathome/fbot_description.git](https://github.com/fbotathome/fbot_description.git)
git clone -b main [https://github.com/fbotathome/fbot_hri.git](https://github.com/fbotathome/fbot_hri.git)
git clone -b main [https://github.com/fbotathome/fbot_manipulator.git](https://github.com/fbotathome/fbot_manipulator.git)
git clone -b main [https://github.com/fbotathome/fbot_navigation.git](https://github.com/fbotathome/fbot_navigation.git)
git clone -b main [https://github.com/fbotathome/fbot_vision.git](https://github.com/fbotathome/fbot_vision.git)
git clone -b main [https://github.com/fbotathome/fbot_webclient.git](https://github.com/fbotathome/fbot_webclient.git)
git clone -b main [https://github.com/fbotathome/fbot_world.git](https://github.com/fbotathome/fbot_world.git)
git clone -b main git@github.com:fbotathome/fbot_intelligence.git

# Repositórios em branches específicas
git clone -b feat/extract_data_vim git@github.com:fbotathome/fbot_behavior.git
git clone -b master git@github.com:fbotathome/fbot_simulation.git
git clone -b ros2 [https://github.com/fbotathome/joy2twist.git](https://github.com/fbotathome/joy2twist.git)

# Repositório externo do braço xArm
git clone -b humble --recursive [https://github.com/xArm-Developer/xarm_ros2.git](https://github.com/xArm-Developer/xarm_ros2.git)

# Pacote interbotix_xsarm_descriptions
git clone -b humble [https://github.com/Interbotix/interbotix_ros_manipulators.git](https://github.com/Interbotix/interbotix_ros_manipulators.git) /tmp/interbotix_tmp
cp -r /tmp/interbotix_tmp/interbotix_ros_xsarms/interbotix_xsarm_descriptions ~/fbot_ws/src/
rm -rf /tmp/interbotix_tmp

```

### 2.3 Ignorar pacotes de controle incompatíveis

Evita erros de compilação com o Gazebo Classic.

```bash
touch ~/fbot_ws/src/xarm_ros2/xarm_gazebo/COLCON_IGNORE
touch ~/fbot_ws/src/xarm_ros2/xarm_gazebo/include/xarm_gazebo/COLCON_IGNORE
touch ~/fbot_ws/src/xarm_ros2/thirdparty/realsense_gazebo_plugin/COLCON_IGNORE
touch ~/fbot_ws/src/xarm_ros2/thirdparty/realsense_gazebo_plugin/include/realsense_gazebo_plugin/COLCON_IGNORE

```

### 2.4 Instalar Dependências Python e Áudio

```bash
sudo apt install -y python3-pyaudio redis-server
sudo systemctl enable --now redis-server

pip3 install \
redis termcolor jellyfish "numpy==1.24.4" scikit-learn transforms3d \
ros2-numpy fpdf pyserial dynamixel_sdk smbus sounddevice playsound \
realtimeSTT "litellm==1.75.5.post1" openai google-generativeai anthropic \
python-dotenv Pillow requests PyYAML transformations open3d ultralytics \
transformers accelerate

# Smolagents customizado
pip3 install git+[https://github.com/butia-bots/smolagents.git](https://github.com/butia-bots/smolagents.git)

```

## Parte 3: Configuração da Simulação

### 3.1 Importar o Pacote de Simulação Preparado

Para evitar configurações manuais nos URDFs e nos parâmetros do DWB/Nav2, transfira o arquivo `fbot_sim_pack.tar.gz` e extraia na máquina de desenvolvimento:

```bash
tar -xzvf ~/fbot_sim_pack.tar.gz -C ~/

```

*(Certifique-se de que o arquivo fbot_sim_pack.tar.gz está na raiz do seu usuário ~/ antes de rodar o comando)*

### 3.2 Variáveis de Ambiente e Compilação

```bash
# Adiciona isolamento de rede DDS
echo -e "\nexport ROS_DOMAIN_ID=42\nexport ROS_LOCALHOST_ONLY=1" >> ~/.bashrc
[ -f ~/.zshrc ] && echo -e "\nexport ROS_DOMAIN_ID=42\nexport ROS_LOCALHOST_ONLY=1" >> ~/.zshrc

# Atualiza dependências e compila
cd ~/fbot_ws
source /opt/ros/humble/setup.bash
sudo rosdep init 2>/dev/null || true
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install

```

## Parte 4: Executando a Prova Inspection

Com tudo compilado, abra 3 terminais separados:

**Terminal 1 - Abrir o Simulador (Gazebo + Nav2 + RViz2 + RQt)**

```bash
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/fbot_ws/install/setup.bash
ros2 launch fbot_bringup simulation.launch.py

```

*(No RViz2, clique em "2D Pose Estimate" sobre a posição do Bóris na entrada da arena para ativar o AMCL e o Nav2).*

**Terminal 2 - Iniciar o Servidor de Poses**

```bash
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/fbot_ws/install/setup.bash
ros2 launch fbot_bringup world.launch.py config_file_name:=pose_inspection

```

**Terminal 3 - Executar a Máquina de Estados**

```bash
source ~/.bashrc
source /opt/ros/humble/setup.bash
source ~/fbot_ws/install/setup.bash
ros2 run fbot_behavior inspection

```

```

### Como incluir isso no Git
Como estamos trabalhando em uma documentação, o ideal é colocá-la na mesma branch da simulação antes de levar tudo para a *main*. 

1. Se você for na pasta do `fbot_bringup`, o arquivo `README.md` original da equipe deve estar lá.
2. Abra ele com `gedit ~/fbot_ws/src/fbot_bringup/README.md` e cole esse conteúdo.
3. Se quiser adicionar ao Git da equipe, basta rodar:
```zsh
cd ~/fbot_ws/src/fbot_bringup
git add README.md
git commit -m "docs: add full automated installation guide in markdown"
git push origin feat/sim_inspection_fixes

```

