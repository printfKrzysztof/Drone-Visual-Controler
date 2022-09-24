# HOW TO RUN

**All of this needs to be in catkin_ws/src directory!!**

Launch simulation in different terminal than roslaunch file.

```
roslaunch dvc_sim mylanuch.launch
```

just like 

```
roslaunch drone_sim runaway3.launch
```
but first build package

# MAKE ALL CMAKES INTO ONE PACKAGE TO HAVE BETTER WORKFLOW 

```
mode guaided
```
## OKAY FUN NOW

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```

# TO FIX: 
LOW FPS

change the angle of camera attack