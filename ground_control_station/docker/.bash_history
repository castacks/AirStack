tmux a
tmux ls
cd ~/ros_ws
ros2 launch gcs_bringup gcs.launch.xml 
cws
bws
sws
bws --packages-select gcs_bringup
mosquitto_sub -h localhost -t healthcheck -u airlab
mosquitto_sub -h localhost -t to_tak -u airlab
