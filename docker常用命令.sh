docker ps -a

docker rm d19ea9a692e3
docker rm mrslam

docker exec -it 8eab18cd3ac4 /bin/bash
source /opt/ros/noetic/setup.bash


# run docker

xhost +local:docker
MYUID=$(id -u); MYGID=$(id -g)

docker rm -f mrslam 2>/dev/null

docker run -it \
  --name mrslam \
  --net=host \
  --gpus all \
  --user $MYUID:$MYGID \
  -e NVIDIA_DRIVER_CAPABILITIES=all \
  -e DISPLAY=$DISPLAY \
  -e QT_X11_NO_MITSHM=1 \
  -e XDG_RUNTIME_DIR=/run/user/$MYUID \
  -e DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$MYUID/bus" \
  -e HOME=/tmp/home \
  -e ROS_HOME=/tmp/home/.ros \
  -e LD_PRELOAD= \
  -v /run/user/$MYUID:/run/user/$MYUID \
  -v /tmp/.X11-unix:/tmp/.X11-unix \
  -v /etc/passwd:/etc/passwd:ro \
  -v /etc/group:/etc/group:ro \
  -v /home/cyw/CYW/mapping/MR_SLAM:/home/cyw_local/MR_SLAM \
  maverickp/mrslam:noetic bash --noprofile --norc
