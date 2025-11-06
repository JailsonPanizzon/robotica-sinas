docker build -t ros-gazebo:humble-ros .

xhost +local:docker

docker run -it --rm --env="DISPLAY=$DISPLAY" --env="QT_X11_NO_MITSHM=1" --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --volume="$HOME/.Xauthority:/root/.Xauthority:rw" -v "$(pwd):/ros2" --net=host --privileged ros-gazebo:humble-ros

ros2 launch simple_gazebo_robot sim.launch.py
