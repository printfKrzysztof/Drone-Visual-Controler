# Running roslaunch files (simulation and programs at once)
Lets base our example roslaunch upon [TreeSeeker](https://github.com/Pigwomaniak/tree_seeker) from [Pigwomaniak](https://github.com/Pigwomaniak) and [tom1322s](https://github.com/tom1322s) inspired by [Inteligent Quads](https://github.com/Intelligent-Quads)
## Getting repozitory



Lets get all packages that tree_seeker uses:

Starting with drone sim.
```
cd catkin_ws/src
git clone https://github.com/Pigwomaniak/drone_sim.git
echo "GAZEBO_MODEL_PATH=${GAZEBO_MODEL_PATH}:$HOME/catkin_ws/src/drone_sim/models" >> ~/.bashrc
source ~/.bashrc
```
Then ball_dropper
```
cd catkin_ws/src
git clone https://github.com/Pigwomaniak/ball_dropper.git
source ~/.bashrc
```
And finally tree_seeker
```
cd ./catkin_ws/src
git clone https://github.com/Pigwomaniak/tree_seeker
cd ./tree_seeker
```

## Running simulation:
In terminal 1:
```
roslaunch mission_commander trees_sim.launch
```
In terminal 2:
```
roslaunch drone_sim runway3.launch
```
In terminal 3:
```
./startsitl.sh
```
In terminal 4:
```
cd MissionPlanner-latest/
mono MissionPlanner.exe
```
<!-- 
In terminal 5:
```
roslaunch mavros apm.launch fcu_url:=udp://127.0.0.1:14551@14555
``` 
-->

Then just take off and thats all
