# Fazer o build do container docker

docker build -t ros-gazebo:humble-ros .

# Ativar a conecção gráfica, se necesssário no linux

xhost +local:docker

# Rodar o container com acesso aos arquivos na pasta do projeto

docker run -it --rm --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v "$(pwd):/ros2" --net=host --privileged ros-gazebo:humble-ros

# Acessar um container rodando

docker ps
docker exec -it {ID_TERMINAL} bash

# Buildar todos os pacotes instalados

colcon build
source install/setup.bash

# Rodar a simulação

ros2 launch simple_gazebo_robot sim.launch.py
ros2 run simple_gazebo_robot keyboard_node

# Rodar o bridge do ROS para o gazebo

ros2 run ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist]ignition.msgs.Twist
