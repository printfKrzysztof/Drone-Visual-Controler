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

From catkin_ws directory run:

```
wstool init ~/catkin_ws/src
rosinstall_generator --upstream mavros | tee /tmp/mavros.rosinstall
rosinstall_generator mavlink | tee -a /tmp/mavros.rosinstall
wstool merge -t src /tmp/mavros.rosinstall
wstool update -t src
rosdep install --from-paths src --ignore-src --rosdistro `echo $ROS_DISTRO` -y --os=ubuntu:focal
catkin build
```

Resorce again

```
echo "source ~/catkin_ws/devel/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo ~/catkin_ws/src/mavros/mavros/scripts/install_geographiclib_datasets.sh
```