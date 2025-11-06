docker build -t ros-gazebo:humble-ros .

xhost +local:docker

docker run -it --rm --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v "$(pwd):/ros2" --net=host --privileged ros-gazebo-sinas:humble-sinapse

<!-- criar primeiro pacote como simulação -->

ros2 pkg create --build-type ament_cmake basic_sim \
 --dependencies rclpy gazebo_ros xacro

<!-- PARA ARM -->

cat <<'EOF' | tee /etc/apt/sources.list
deb http://mirror.kumi.systems/ubuntu-ports jammy main restricted universe multiverse
deb http://mirror.kumi.systems/ubuntu-ports jammy-updates main restricted universe multiverse
deb http://mirror.kumi.systems/ubuntu-ports jammy-security main restricted universe multiverse
EOF

apt-get update

sudo apt-get install ros-humble-ros-gz -y --fix-missing

sudo apt update
sudo apt install -y libignition-gazebo6-dev ignition-gazebo6

<!-- RUN -->

ros2 pkg create --build-type ament_cmake simple_gazebo_robot

<!-- aidionar no CMakeListis.txt -->

install(
DIRECTORY launch
DESTINATION share/${PROJECT_NAME}/
)

install(
DIRECTORY worlds models
DESTINATION share/${PROJECT_NAME}/
)

install(
DIRECTORY include/
DESTINATION include/
)

colcon build && source install/setup.bash

ros2 launch simple_gazebo_robot sim.launch.py
