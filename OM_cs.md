
## OpenManipulator Cheat Sheet

______________________________________________________________________________
#### - Always    
- `source ~/.bashrc`  
- `. /root/catkin_ws/devel/setup.bash`  

Both combined:   
- `source ~/.bashrc && . /root/catkin_ws/devel/setup.bash`
  
Finally run `ifconfig` and set the `ROS_MASTER_URI`  


```bash 
export ROS_MASTER_URI='http://192.168.0.150:11311'  
export ROS_MASTER_URI='http:// :11311'  
```
###### _TODO: Write a program to automatically set this_
______________________________________________________________________________
#### - Creating a new Lib Real Sense application

Duplicate a folder in the Examples/C/ folder
Edit this CMakeLists.txt to match the information of the project you are trying to make... should be three or four lines that you have the changel.
    All these reflect and should match the code changes in your project

Next you need to add the path of your new project to the CMakeLists.txt that exists in /examples folder

For example if you made a new project in a folder named distance-xyz
you would have to edit and add the path of distance-xyz to the CMakeLists that exists in the examples/folder.. 
`add_executable(C/distance-xyz)`

ADB push accordingly


outer CMakeLists.txt
`adb push /home/kav/Documents/librealsense/examples/CMakeLists.txt /data/rs/librealsense/examples/.`
inner Folder
`adb push /home/kav/Documents/librealsense/examples/C/distance-xyz /data/rs/librealsense/examples/C/.`  
  
then cd to the build folder

make -j7

sudo make install

then type the execuatble and it should work.

#### - Object detection
On windows side: files are located in C:\dev\rs_object_detection
On rb5 side: files are lcated in /home/rs_object_detection

adb push rs_object_detection /home/.

______________________________________________________________________________ 	
#### - KeyboardTeleop  
##### Open four terminals 
- `roscore`  

Need to run the OpenManipulator controller [-WikiLink-](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller) 

- `roslaunch open_manipulator_controller open_manipulator_controller.launch`   
- `cd /root/catkin_ws/src/Grasp-pose-detection-master/build && ./tflite-demo-app-ros1` 
- Next run the keyboard teleop itself [-WikiLink-](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_operation/#teleoperation)  
- `roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch`

- `roslaunch my_open_manipulator_teleop my_open_manipulator_teleop_keyboard.launch`


- .cpp  
`adb push /home/kav/Documents/openmanip/my_open_manipulator_teleop/src/open_manipulator_teleop_keyboard.cpp /root/catkin_ws/src/my_open_manipulator_teleop/src/.`  
  
- .h
`adb push /home/kav/Documents/openmanip/my_open_manipulator_teleop/include/open_manipulator_teleop/open_manipulator_teleop_keyboard.h /root/catkin_ws/src/my_open_manipulator_teleop/include/open_manipulator_teleop/.`  
  
- .txt
`adb push /home/kav/Documents/openmanip/my_open_manipulator_teleop/CMakeLists.txt /root/catkin_ws/src/my_open_manipulator_teleop/.`

- .xml
`adb push /home/kav/Documents/openmanip/my_open_manipulator_teleop/package.xml /root/catkin_ws/src/my_open_manipulator_teleop/.`



Now for making a duplicate teleop code 

adb pull /root/catkin_ws/src/open_manipulator/open_manipulator_teleop .

adb push /home/kav/Documents/openmanip/non_blocking_teleop /root/catkin_ws/src/.

go to root/catkin_ws/build

cmake .
make


adb push open_manipulator_teleop_keyboard.cpp /root/catkin_ws/src/non_blocking_teleop/src/.

adb push non_blocking_teleop /root/catkin_ws/src/.


roslaunch non_blocking_teleop non_blocking_teleop.launch

______________________________________________________________________________

#### - Melan's Demo (AR Detection & Pick and Place)  
##### Open four terminals  
- `roscore`  

Need to run the OpenManipulator controller [-WikiLink-](https://emanual.robotis.com/docs/en/platform/openmanipulator_x/ros_controller_package/#launch-controller)  

- `roslaunch open_manipulator_controller open_manipulator_controller.launch`  
- `cd /root/catkin_ws/src/Grasp-pose-detection-master/build && ./tflite-demo-app-ros1`
- `roslaunch open_manipulator_pick_and_place open_manipulator_pick_and_place.launch`  
  

adb pull /root/catkin_ws/src/Grasp-pose-detection-master .














  
```txt
Make sure to open two separate terminals
Also position the arm in the correct off position

First run this command in one of the terminals
ros2 launch open_manipulator_x_controller open_manipulator_x_controller.launch.py usb_port:=/dev/ttyACM0
Then in the next terminal run:
ros2 run open_manipulator_x_teleop open_manipulator_x_teleop_keyboard



changed to the correct ips
source ~/.bashrc
. /root/catkin_ws/devel/setup.bash
Ifconfig 
Set your ros_master_uri to the correct ip
export ROS_MASTER_URI='http://192.168.0.174:11311'  
export ROS_MASTER_URI='http:// :11311'  
export ROS_MASTER_URI='http:// :11311'  
export ROS_MASTER_URI='http:// :11311'  
When entering a new python file, you need to make it an executable
Chmod +x  /root/catkin_ws/devel/lib/open_manipulator_controller/<name of python file>
chmod +x  /root/catkin_ws/devel/lib/open_manipulator_controller/rb5_moveit_open_manipulator_dup.py  
  
  
Adb push <python file> /root/catkin_ws/devel/lib/open_manipulator_controller/

Adb push rb5_moveit_open_manipulator_dup.py /root/catkin_ws/devel/lib/open_manipulator_controller/.

Controller launch is the file to change
Adb push <controller launch> /root/catkin_ws/src/open_manipulator/open_manipulator_controller/launch/
Adb push open_manipulator_controller_dup.launch /root/catkin_ws/src/open_manipulator/open_manipulator_controller/launch/.

roslaunch open_manipulator_controller open_manipulator_controller_dup.launch use_moveit:=true

f


roslaunch open_manipulator_pick_and_place open_manipulator_pick_and_place.launch








1st number turns counter clockwise. Base motor closest to the base plate
1.5 = 90degrees

2nd number moves the arm up and down… it’s the 2nd joint from the bottom
-0.05 is very close to the off start position aka close to the base plate

3rd number moves the thir joint up and down.. 

 roslaunch open_manipulator_teleop open_manipulator_teleop_keyboard.launch

roslaunch o


 adb pull /root/catkin_ws/src/open_manipulator/open_manipulator_teleop .

 adb push my_open_manipulator_teleop /root/catkin_ws/src/.

 roslaunch my_open_manipulator_teleop open_manipulator_teleop_keyboard.launch






roslaunch my_open_manipulator_teleop my_open_manipulator_teleop_keyboard.launch

```


adb push open_manipulator_pick_and_place_1.cpp /root/catkin_ws/src/open_manipulator_pick_and_place/src/.


adb pull /data/rs/librealsense .