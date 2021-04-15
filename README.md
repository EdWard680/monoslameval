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

### lsd_slam

Note that lsd_slam depends on very outdated versions of Ubuntu and ROS, and can't really run on the same system as ORB_SLAM2.

## Running the datasets with the algorithms

We attempted to set up launch files that could run the datasets for both algorithms, however roslaunch inexplicably doesn't want to start the lsd_slam nodes on ROS indigo, so a separate shell script for starting up the lsd_slam stack using `rosrun` is included.

To test an algorithm against a dataset, simply run
```
roslaunch test_dataset.launch dataset:=<dataset folder> algorithm:=<algorithm launch file>
```

This should work out of the box for the datasets we produced with the M-Bot, however to run on another dataset, you can specify `dataset_image_topic` to set the image topic which is produced by your bag file. If the topic is of the raw image already, also set `decompress:=false`.

This will launch rqt_bag on the bag file. Right-click the camera topic (`/camera/compressed` for the M-Bot datasets) and check the box that says 'publish'. Then you can hit play and you should see the image appear in the viewer window for the algorithm you're running.

Note for ORB_SLAM2, for some reason the window doesn't resize properly. Just close the image viewer window on ORB_SLAM2 and it will re-open
