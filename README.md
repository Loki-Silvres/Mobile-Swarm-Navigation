Instructions for running the setup locally :
1) clone the repo before every use
2) add following line to bashrc :
  export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/turtlebot3_ws/src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/modelsexport TURTLEBOT3_MODEL=waffle_pi
3) go to src/turtlebot3/turtlebot3_simulations/turtlebot3_gazebo/launch
4) run python3 xacro_launch.py
