# Running roslaunch files (simulation and programs at once)

## Getting repozitory

Running [TreeSeeker](https://github.com/Pigwomaniak/tree_seeker) from [Pigwomaniak](https://github.com/Pigwomaniak) and [tom1322s](https://github.com/tom1322s) inspired by [Inteligent Quads](https://github.com/Intelligent-Quads):

Get the drone_sim first:
```
cd catkin_ws/src
git clone https://github.com/Pigwomaniak/drone_sim.git
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/drone_sim/models" >> ~/.bashrc
source ~/.bashrc
```
also ball_dropper
```
cd catkin_ws/src
git clone https://github.com/Pigwomaniak/ball_dropper.git
source ~/.bashrc
```

```
cd ./catkin_ws/src
git clone https://github.com/Pigwomaniak/tree_seeker
cd ./tree_seeker
```

```
roslaunch mission_commander trees_sim.launch
```
