# Pra_Vale

Esse repositório contém o pacote ROS e os arquivos V-REP necessários para o funcioamento da submissão da equipe Pra Vale no desafio ROSI.

A descrição sobre a competição, assim como os arquivos V-REP oficiais, podem ser encontrados no seguinte repositório: https://github.com/filRocha/rosiChallenge-sbai2019

# Descrição do repositório:
Este respositório consiste da mesma estruta de um pacote ROS. A organização das pastas se dá da seguinte forma:

- `config` - Contém o arquivo com os parametros da simulação

- `launch` - Contêm os arquivos do tipo .launch do ROS

- `msg` - Contêm os arquivos do tipo .launch do ROS necessários para a comunição com a simulação

- `resources` - Contêm o regulamento e o banner da competição

- `script` - Códigos criados pela equipe para o processamento de dados da simulação e controle do robô

- `urdf` - Comtém o modelo URDF do robô ROSI

- `vrep_content` - Contêm os arquivos V-REP necessários para a simulação


# Instalação
Toda a programção foi feita no SO **Ubuntu 18.2**, junto com o **ROS Melodic** e **VREP 3.6.2 (rev.0)**.

# 1. Instalando ROS Melodic
A explicação detalhada sobre como instalar o ROS Melodic pode ser encontrada no seguinte link:
http://wiki.ros.org/melodic/Installation/Ubuntu

Caso queira somente copiar e colar os comandos necessários, sem saber os detalhes, pode seguir os passos abaixo:
``` 
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654
``` 

``` 
sudo apt update
sudo apt install ros-melodic-desktop-full
``` 

``` 
sudo rosdep init
rosdep update
``` 

``` 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
source ~/.bashrc
``` 

``` 
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential
``` 

# 2. Configurando Catkin Workspace
Do mesmo modo que o primeiro itêm, a explicação detelhada de como configurar o workspace pode ser encontrado no seguinte link:
http://wiki.ros.org/catkin/Tutorials/create_a_workspace

Caso queira somente os comandos, eles estão listados abaixo:
``` 
sudo apt-get install ros-melodic-catkin python-catkin-tools
``` 

``` 
cd ~
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/
``` 

``` 
catkin_make
source devel/setup.bash
catkin init
``` 

# 3. Installing V-REP
**3.1** Download **V-REP PRO EDU V3.6.2 rev0** from the Coppelia Robotics website: http://www.coppeliarobotics.com/downloads.html


**3.2** Unzip it (preferentially) to your **home** folder and rename the folder as `vrep`.


**3.3** Add V-REP folder location to your `.bashrc`, an alias to run it, and source the `.bashrc` again: 
```
echo "export VREP_ROOT='<path_to_your_vrep_folder>'" >> $HOME/.bashrc
echo "alias vrep=$VREP_ROOT/vrep.sh" >> ~/.bashrc
source $HOME/.bashrc
```

# 4. Installing competition base code
The detailed explanation about how to install the ROSI challenge base code is in the following link:https://github.com/filRocha/rosiChallenge-sbai2019

In case you only want the terminal commands, they are listed below:
```
echo "export ROS_CATKIN_WS='<path_to_your_catkin_ws_folder>'" >> $HOME/.bashrc
echo "source $ROS_CATKIN_WS/devel/setup.bash" >> $HOME/.bashrc
source $HOME/.bashrc
```
``` 
cd ~/catkin_ws/src
``` 

``` 
git clone https://github.com/filRocha/sbai2019-rosiDefy rosi_defy
``` 

``` 
sudo apt install python-catkin-tools xsltproc ros-$ROS_DISTRO-brics-actuator ros-$ROS_DISTRO-tf2-sensor-msgs ros-$ROS_DISTRO-joy ros-$ROS_DISTRO-joint-state-publisher
``` 

``` 
cd $ROS_CATKIN_WS
catkin clean
catkin build
source $HOME/.bashrc
``` 

``` 
echo -e "rosi_defy/ManipulatorJoints\nrosi_defy/RosiMovement\nrosi_defy/RosiMovementArray\nrosi_defy/HokuyoReading" >> $ROS_CATKIN_WS/src/vrep_ros_interface/meta/messages.txt
``` 

# 4.1 Set base code dependencies:
Add "<depend>rosi_defy</depend>" to 'catkin_ws/src/vrep_ros_interface/package.xml'

Add `rosi_defy` package dependence on the `catkin_ws/src/vrep_ros_interface/CMakeLists.txt`:
```
set(PKG_DEPS
  ... (many many other packages)
  rosi_defy
)
```

``` 
cd ~/catkin_ws
catkin build
``` 

``` 
cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosInterface.so $VREP_ROOT
cp $ROS_CATKIN_WS/devel/lib/libv_repExtRosVelodyne.so $VREP_ROOT
``` 

# 5. Cloning Pra Vale submission
``` 

``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 


``` 
``` 



