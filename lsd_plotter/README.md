# LSD-slam running instructions

## Installation:
### 1. Create your own workspace <yourworkspace>
### 2. Copy these two packages under /`<yourworkspace>`/src
### 3. Run the following commands:
```
cd ~/<yourworkspace>
catkin_make
source ~/<yourworkspace>/devel/setup.bash
```
### Make sure everytime you use these two packages, source it in the new terminal, or you can run:
```
echo "source ~/<yourworkspace>/devel/setup.bash" >> ~/.bashrc
```
to add it to '.bashrc' for future running.

## Running:

### For extracting images frame by frame from our own datasets from the m-bot:
#### 1. Modify the 'extractImages.py' under /extractimagefrombag/src to your IO settings as you want
#### 2. Locate your '.bag' file's folder
#### 3. Open the terminal and Run: (or use 'cd' to locate your '.bag' file's folder)
```
rosrun extractimagefrombag extractImages.py
```

### For extracting messages from certain topic from the '.bag' file:
#### 1. Modify the 'generatetxtdata.py' under /generateplotdata/src to your extraction settings as you want
#### 2. Locate your '.bag' file's folder
#### 3. Open the terminal and Run: (or use 'cd' to locate your '.bag' file's folder)
```
rosrun generateplotdata generatetxtdata.py
```
