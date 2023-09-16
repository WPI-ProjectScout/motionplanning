# motionplanning

## Install Anaconda
## Install ROS2 Foxy
https://docs.ros.org/en/foxy/Installation.html
## Install Google Chrome

## Install Carla from leaderboard (section 1)
* [https://leaderboard.carla.org/get_started/#12-get-the-leaderboard-and-scenario-runner](https://leaderboard.carla.org/get_started/#1-system-setup)]
* Follow all steps including the scenario runner and leaderboard in section 1


### Check your install
python3 -c 'import carla;print("Success")' # python3
* Make sure to run
  * <code> conda activate py37 </code>
May need to fix the line in .bashrc:
<code>
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.13-py3.7-linux-x86_64.egg":${PYTHONPATH}
</code>
to
<code>
export PYTHONPATH="${CARLA_ROOT}/PythonAPI/carla/":"${SCENARIO_RUNNER_ROOT}":"${LEADERBOARD_ROOT}":"${CARLA_ROOT}/PythonAPI/carla/dist/carla-0.9.14-py3.7-linux-x86_64":${PYTHONPATH}
</code>

  
## Setup the ROS Bridge
* First, run command to install rosdep:
* <code>sudo apt install python3-rosdep2</code>
* https://carla.readthedocs.io/projects/ros-bridge/en/latest/ros_installation_ros2/

