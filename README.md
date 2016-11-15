# sl_crazyflie

## Installation
Change to your catkin workspace src directory, e.g:
```
cd ~/catkin_ws/src
```
Checkout this repository:
```
git clone https://github.com/smARTLab-liv/sl_crazyflie
```
Checkout dependencies:
```
git clone https://github.com/whoenig/crazyflie_ros
git clone https://github.com/ros-drivers/mocap_optitrack (optional, depending on mocap system)
```
catkin_make your workspace:
```
cd ~/catkin_ws/src
catkin_make
```
## Preparation
If you are not using mocap_optitrack, go to the sl_crazyflie_demo/launch folder.
Change the value of the "cf_pose_topic" parameter, the subscriber to this topic 
expects a PoseStamped which represents the crazyflie pose in the World-frame. 

## Run
Pair the ps3-controller.
Start the mocap-node, in our case:
```
roslaunch mocap_optitrack mocap.launch
```
Start our controller:
```
roslaunch sl_crazyflie_demo ps3_teleop.launch
```

## PS3 Controls
Start-Button = take-off
Arrow-Buttons = change target pose in x and y (+/- 10cm)
Triangle-button = increase target pose z value (+ 10cm)
Triangle-button = decrease target pose z value (- 10cm)


## License
The MIT License (MIT)
Copyright (c) 2016 smARTLab, University of Liverpool

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
