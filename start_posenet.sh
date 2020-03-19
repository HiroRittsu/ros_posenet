cd $(cd $(dirname $0);pwd)/posenet-models
python3 -m http.server &
roslaunch ros_posenet posenet.launch
