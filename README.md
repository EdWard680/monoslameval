# monoslameval
Team 9 Final Project from EECS568

## Structure of this repository

This repo references a few other repositories which implement the SLAM algorithms used.
Use the following command to clone this repository as well as the dependent submodules

```
git clone --recursive https://github.com/EdWard680/monoslameval.git
```

Each dataset should go inside its own folder as a bag. Since these bags are large they are not included in this repository, but a link to download the bag is available inside the README within each dataset folder.

These folders also contain the requisite calibration files for each of the algorithms.

## Building the SLAM implementations

Each sub-repo includes instructions for how to build their code which you should follow to build and/or install their code.

### ORB_SLAM2

To build this you should just need to run
```
./build.sh
./build_ros.sh
```
in the ORB_SLAM2 folder as it says. Note that there seems to be a bug on some systems where usleep is not defined, the fix to this is to add
```
#include <unistd.h>
```
to the top of `include/System.h` in the ORB_SLAM2 directory.

### LSD-SLAM: Large-Scale Direct Monocular SLAM

Note that lsd_slam depends on very outdated versions of Ubuntu and ROS, and can't really run on the same system as ORB_SLAM2.

#### Installation
##### 1. ROS fuerte + Ubuntu 12.04
Install system dependencies:

    sudo apt-get install ros-fuerte-libg2o liblapack-dev libblas-dev freeglut3-dev libqglviewer-qt4-dev libsuitesparse-dev libx11-dev

Find your ROS package path, copy the 'lsd_slam' folder to your ROS package path.

Compile the two package by typing:

    rosmake lsd_slam

##### 2. ROS indigo + Ubuntu 14.04
**Not using catkin, however fortunately old-fashioned CMake-builds are still possible with ROS indigo.**
For this you need to create a rosbuild workspace (if you don't have one yet), using:

    sudo apt-get install python-rosinstall
    mkdir ~/rosbuild_ws
    cd ~/rosbuild_ws
    rosws init . /opt/ros/indigo
    mkdir package_dir
    rosws set ~/rosbuild_ws/package_dir -t .
    echo "source ~/rosbuild_ws/setup.bash" >> ~/.bashrc
    bash
    cd package_dir

Install system dependencies:

    sudo apt-get install ros-indigo-libg2o ros-indigo-cv-bridge liblapack-dev libblas-dev freeglut3-dev libqglviewer-dev libsuitesparse-dev libx11-dev

Find your ROS package path, copy the 'lsd_slam' folder under your ROS package path.

Compile the two package by typing:

    rosmake lsd_slam

BUG fixing (if error occurs):
If the message like this shows when running command `rosmake lsd_slam`: 'ERROR [gendeps] 1 Finding dependencies for /home/`<yoursettings>`/rosbuild_ws/package_dir/lsd_slam/lsd_slam_viewer/cfg/LSDSLAMViewerParams.cfg', then one may fix the problem according to the following instructions:
```
(1) Open lsd_slam_viewer/cfg/LSDSLAMViewerParams.cfg：
　　　　　　　　　Line 20: scaledDepthVarTH： delete the "'" of the words "point's" and "keyframe's";
　　　　　　　　　Line 21: absDepthVarTH：delete the "'" of the word "point's";
　　　　　　　　　Line 24: cutFirstNKf：delete the "'" of the word "keyframe's".
(2) Open lsd_slam_core/cfg/LSDDebugParams.cfg：
　　　　　　　　　Line 11: plotStereoImages： delete the "'" of the word "what's";
　　　　　　　　　Line 12: plotTracking：delete the "'" of the word "what's";
　　　　　　　　　Line 48: continuousPCOutput：delete the "'" of the word "Keyframe's".
```

## Running the datasets with the algorithms

We attempted to set up launch files that could run the datasets for both algorithms, however roslaunch inexplicably doesn't want to start the lsd_slam nodes on ROS indigo, so a separate shell script for starting up the lsd_slam stack using `rosrun` is included.

To test an algorithm against a dataset, simply run
```
roslaunch test_dataset.launch dataset:=<dataset folder> algorithm:=<algorithm launch file>
```

This should work out of the box for the datasets we produced with the M-Bot, however to run on another dataset, you can specify `dataset_image_topic` to set the image topic which is produced by your bag file. If the topic is of the raw image already, also set `decompress:=false`.

This will launch rqt_bag on the bag file. Right-click the camera topic (`/camera/compressed` for the M-Bot datasets) and check the box that says 'publish'. Then you can hit play and you should see the image appear in the viewer window for the algorithm you're running.

Note for ORB_SLAM2, for some reason the window doesn't resize properly. Just close the image viewer window on ORB_SLAM2 and it will re-open

### LSD-slam running instructions
#### 1. Everytime you need to run LSD-slam with a new window, make sure you correctly enter in the terminal:
```
source ~/rosbuild_ws/setup.bash
```
#### 2. Locate under the folder 'monoslameval'

#### 3. Open the terminal and give the execution ability to the .sh file
```
sudo chmod u+x lsd_slam.sh
```

#### 4. Run the following command to get all pre-programs running:
```
./lsd_slam.sh
```

#### 5. Run the dataset by setting 'algorithm:=external.launch'

#### Everytime you want to change to run a different dataset, make sure the calibration file 'camera.cfg' is correctly selected.
#### Reference link: https://github.com/tum-vision/lsd_slam#313-camera-calibration

# Generating Comparison Plots

A useful tool called `rpg_trajectory_evaluation` is included with this repository. This tool is useful for comparing estimated and ground truth trajectories for the purposes of plotting.

## ORB_SLAM2

After running ORB_SLAM2 it will save the estimated trajectory to a weird location on your computer. To copy it here, run the provided script
```
./get_most_recent_orbslam_trajectory.sh
```

To analyze it, move it into the dataset folder you just ran naming it `stamepd_traj_estimate.yaml`. Now, you can run the evaluation tool on that folder like so
```
python2 rpg_trajectory_evaluation/scripts/analyze_trajectory_single.py <dataset folder>
```

This will generate some analyses and plots in that dataset folder which you can view. Under the hood, this package is running some algorithms to fit the scale of the monocular SLAM trajectory to the ground-truth scale.

## LSD_SLAM
See the folder: `lsd_plotter` for instructions on extracting and comparing the data for lsd_slam
