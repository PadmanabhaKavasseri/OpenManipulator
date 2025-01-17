﻿/*******************************************************************************
* Copyright 2018 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*jj
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Darby Lim, Hye-Jong KIM, Ryan Shim, Yong-Ho Na */

#include "open_manipulator_teleop/open_manipulator_teleop_keyboard.h"
#include <iostream>
#include <unistd.h>

OpenManipulatorTeleop::OpenManipulatorTeleop() :node_handle_(""), priv_node_handle_("~")
{
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  initClient();
  initSubscriber();

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop()
{
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
}

void OpenManipulatorTeleop::initClient()
{
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");
  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}
void OpenManipulatorTeleop::initSubscriber()
{
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
  ar_pose_marker_sub_ = node_handle_.subscribe("/position", 10, &OpenManipulatorTeleop::arPoseMarkerCallback, this);


  //TODO: "gripper/kinematics_pose"
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg)
{
  std::vector<double> temp_angle;
  temp_angle.resize(NUM_OF_JOINT);
  for(std::vector<int>::size_type i = 0; i < msg->name.size(); i ++)
  {
    if(!msg->name.at(i).compare("joint1"))  temp_angle.at(0) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint2"))  temp_angle.at(1) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint3"))  temp_angle.at(2) = (msg->position.at(i));
    else if(!msg->name.at(i).compare("joint4"))  temp_angle.at(3) = (msg->position.at(i));
  }
  present_joint_angle_ = temp_angle;

}

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg)
{
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

void OpenManipulatorTeleop::arPoseMarkerCallback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
  //we are setting ->>>>      present_ar_marker_coordinates_
  auto x = msg->pose.position.x;
  auto y = msg->pose.position.y;
  auto z = msg->pose.position.z;

 
  std::cout << "DEBUG: " <<  x << " " <<  std::endl;


  //if y and z are not in a certain range then ignore them

  //set x regardless cuz i cant figure it out
  present_ar_marker_coordinates_[0] = x;
  //y position
  /*
  //decreasing
    left = -0.07
    middle = -0.043
    right = -0.021
    lets set the range as [0 to -0.1]
  */
  //set if only in a certain range
  if(y<=0 && y>=-0.1){
    present_ar_marker_coordinates_[1] = y;
  }
  // else{
  //   present_ar_marker_coordinates_[1] = 1.1111;
  // }

  //z position
  /*
    close = 0.3;
    far away = 0.7

    so lets set the range as [0,1]
  */
  if(z<=1 && z>=0){
    present_ar_marker_coordinates_[2] = z;

    if(z<0.5){
      std::cout << " close " << std::endl;
    }
    else{
      std::cout << " far " << std::endl;
    }

  }
  // else{
  //   present_ar_marker_coordinates_[2] = 1.1111;
  // }

  // present_ar_marker_coordinates_[2] = z;

  //potential warning... the range could change once the arm moves...

}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle()
{
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose()
{
  return present_kinematic_position_;
}

std::vector<double> OpenManipulatorTeleop::getPresentArMarkerCoordinates()
{
  return present_ar_marker_coordinates_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_from_present_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name = joint_name;
  srv.request.joint_position.position = joint_angle;
  srv.request.path_time = path_time;

  if(goal_joint_space_path_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setToolControl(std::vector<double> joint_angle)
{
  open_manipulator_msgs::SetJointPosition srv;
  srv.request.joint_position.joint_name.push_back(priv_node_handle_.param<std::string>("end_effector_name", "gripper"));
  srv.request.joint_position.position = joint_angle;

  if(goal_tool_control_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time)
{
  open_manipulator_msgs::SetKinematicsPose srv;
  srv.request.planning_group = priv_node_handle_.param<std::string>("end_effector_name", "gripper");
  srv.request.kinematics_pose.pose.position.x = kinematics_pose.at(0);
  srv.request.kinematics_pose.pose.position.y = kinematics_pose.at(1);
  srv.request.kinematics_pose.pose.position.z = kinematics_pose.at(2);
  srv.request.path_time = path_time;

  if(goal_task_space_path_from_present_position_only_client_.call(srv))
  {
    return srv.response.is_planned;
  }
  return false;
}

void OpenManipulatorTeleop::printText()
{
  printf("\n");
  printf("---------------------------\n");
  printf("Control Your OpenManipulator!\n");
  printf("---------------------------\n");
  printf("w : increase x axis in task space\n");
  printf("s : decrease x axis in task space\n");
  printf("a : increase y axis in task space\n");
  printf("d : decrease y axis in task space\n");
  printf("z : increase z axis in task space\n");
  printf("x : decrease z axis in task space\n");
  printf("\n");
  printf("y : increase joint 1 angle\n");
  printf("h : decrease joint 1 angle\n");
  printf("u : increase joint 2 angle\n");
  printf("j : decrease joint 2 angle\n");
  printf("i : increase joint 3 angle\n");
  printf("k : decrease joint 3 angle\n");
  printf("o : increase joint 4 angle\n");
  printf("l : decrease joint 4 angle\n");
  printf("\n");
  printf("g : gripper open\n");
  printf("f : gripper close\n");
  printf("       \n");
  printf("1 : init pose\n");
  printf("2 : home pose\n");
  printf("3 : home pose\n");
  printf("       \n");
  printf("q to quit\n");
  printf("---------------------------\n");

  printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
         getPresentJointAngle().at(0),
         getPresentJointAngle().at(1),
         getPresentJointAngle().at(2),
         getPresentJointAngle().at(3));
  printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
         getPresentKinematicsPose().at(0),
         getPresentKinematicsPose().at(1),
         getPresentKinematicsPose().at(2));
  printf("Present AR Marker Coordinates X: %.3lf Y: %.3lf Z: %.3lf\n",
          getPresentArMarkerCoordinates().at(0),
          getPresentArMarkerCoordinates().at(1),
          getPresentArMarkerCoordinates().at(2));
  printf("---------------------------\n");

}

void OpenManipulatorTeleop::setGoal(char ch)
{
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);
  std::vector<double> goalJoint; goalJoint.resize(4, 0.0);

  if(ch == 'w' || ch == 'W')
  {
    printf("input : w \tincrease(++) x axis in task space\n");
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 's' || ch == 'S')
  {
    printf("input : s \tdecrease(--) x axis in task space\n");
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'a' || ch == 'A')
  {
    printf("input : a \tincrease(++) y axis in task space\n");
    goalPose.at(1) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'd' || ch == 'D')
  {
    printf("input : d \tdecrease(--) y axis in task space\n");
    goalPose.at(1) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'z' || ch == 'Z')
  {
    printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'y' || ch == 'Y')
  {
    printf("input : y \tincrease(++) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'h' || ch == 'H')
  {
    printf("input : h \tdecrease(--) joint 1 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1"); goalJoint.at(0) = -JOINT_DELTA;
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'u' || ch == 'U')
  {
    printf("input : u \tincrease(++) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'j' || ch == 'J')
  {
    printf("input : j \tdecrease(--) joint 2 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2"); goalJoint.at(1) = -JOINT_DELTA;
    joint_name.push_back("joint3");
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'i' || ch == 'I')
  {
    printf("input : i \tincrease(++) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'k' || ch == 'K')
  {
    printf("input : k \tdecrease(--) joint 3 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3"); goalJoint.at(2) = -JOINT_DELTA;
    joint_name.push_back("joint4");
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'o' || ch == 'O')
  {
    printf("input : o \tincrease(++) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }
  else if(ch == 'l' || ch == 'L')
  {
    printf("input : l \tdecrease(--) joint 4 angle\n");
    std::vector<std::string> joint_name;
    joint_name.push_back("joint1");
    joint_name.push_back("joint2");
    joint_name.push_back("joint3");
    joint_name.push_back("joint4"); goalJoint.at(3) = -JOINT_DELTA;
    setJointSpacePathFromPresent(joint_name, goalJoint, PATH_TIME);
  }

  else if(ch == 'g' || ch == 'G')
  {
    printf("input : g \topen gripper\n");
    std::vector<double> joint_angle;

    joint_angle.push_back(0.01);
    setToolControl(joint_angle);
  }
  else if(ch == 'f' || ch == 'F')
  {
    printf("input : f \tclose gripper\n");
    std::vector<double> joint_angle;
    joint_angle.push_back(-0.01);
    setToolControl(joint_angle);
  }

  else if(ch == '2')
  {
    printf("input : 2 \thome pose\n");
    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;

    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
    joint_name.push_back("joint3"); joint_angle.push_back(0.35);
    joint_name.push_back("joint4"); joint_angle.push_back(0.70);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '1')
  {
    printf("input : 1 \tinit pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.0);
    joint_name.push_back("joint3"); joint_angle.push_back(0.0);
    joint_name.push_back("joint4"); joint_angle.push_back(0.0);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
  else if(ch == '3')
  {
    printf("input : 3 \tcustom pose\n");

    std::vector<std::string> joint_name;
    std::vector<double> joint_angle;
    double path_time = 2.0;
    joint_name.push_back("joint1"); joint_angle.push_back(0.0);
    joint_name.push_back("joint2"); joint_angle.push_back(0.5);
    joint_name.push_back("joint3"); joint_angle.push_back(0.5);
    joint_name.push_back("joint4"); joint_angle.push_back(0.5);
    setJointSpacePath(joint_name, joint_angle, path_time);
  }
}

void OpenManipulatorTeleop::restoreTerminalSettings(void)
{
    tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(void)
{
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "open_manipulator_teleop");

  OpenManipulatorTeleop openManipulatorTeleop;

  ROS_INFO("OpenManipulator teleoperation using keyboard start");
  openManipulatorTeleop.disableWaitingForEnter();



  ros::spinOnce();

  std::vector<double> goalPose;  goalPose.resize(3, 0.0);

  //initialize arm position
  printf("input : 2 \thome pose\n");
  std::vector<std::string> joint_name;
  std::vector<double> joint_angle;
  double path_time = 2.0;

  joint_name.push_back("joint1"); joint_angle.push_back(0.0);
  joint_name.push_back("joint2"); joint_angle.push_back(-1.05);
  joint_name.push_back("joint3"); joint_angle.push_back(0.35);
  joint_name.push_back("joint4"); joint_angle.push_back(0.70);
  openManipulatorTeleop.setJointSpacePath(joint_name, joint_angle, path_time);


  // goalPose.at(0) = DELTA;
  // openManipulatorTeleop.setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);

  while(ros::ok()){

    //probably not the best run time... and resource management. Fix later.
    goalPose.resize(3, 0.0);


    //might want to add quit code into this from older code module

    ros::spinOnce();

    printf("---------------------------\n"); 
    printf("Present Joint Angle J1: %.3lf J2: %.3lf J3: %.3lf J4: %.3lf\n",
           openManipulatorTeleop.getPresentJointAngle().at(0),
           openManipulatorTeleop.getPresentJointAngle().at(1),
           openManipulatorTeleop.getPresentJointAngle().at(2),
           openManipulatorTeleop.getPresentJointAngle().at(3));
    printf("Present Kinematics Position X: %.3lf Y: %.3lf Z: %.3lf\n",
           openManipulatorTeleop.getPresentKinematicsPose().at(0),
           openManipulatorTeleop.getPresentKinematicsPose().at(1),
           openManipulatorTeleop.getPresentKinematicsPose().at(2));
    printf("Present AR Marker Coordinates X: %.3lf Y: %.3lf Z: %.3lf\n",
            openManipulatorTeleop.getPresentArMarkerCoordinates().at(0),
            openManipulatorTeleop.getPresentArMarkerCoordinates().at(1), 
            openManipulatorTeleop.getPresentArMarkerCoordinates().at(2));
    printf("---------------------------\n");



    //this code should get turned into another function


    // z->AR pose is the x->OpenManip
    // if(openManipulatorTeleop.getPresentArMarkerCoordinates().at(2) > 0.37){
    //   std::cout << "Far -> Moving in +Z direction " << std::endl;

    //   // ar pose z is x coordinate /direction for open manip
    //   goalPose.at(0) = DELTA;
    //   openManipulatorTeleop.setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
    // }
    // else if(openManipulatorTeleop.getPresentArMarkerCoordinates().at(2) < 0.3){
    //   std::cout << "Too Close -> Moving in -Z direction " << std::endl;
    //   goalPose.at(0) = -DELTA;
    //   openManipulatorTeleop.setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);

    // }
    // else{
    //   std::cout << "Object is a safe distance away" << std::endl;
    // }


    // //this is the same 
    // //y position
    // /*
    //   left = -0.07
    //   middle = -0.043
    //   right = -0.021
    //   lets set the range as [0 to -0.1]
    // */

    // if(openManipulatorTeleop.getPresentArMarkerCoordinates().at(1) < -0.037){
    //   std::cout << "Object is on left side" << std::endl;
    // }
    // else if(openManipulatorTeleop.getPresentArMarkerCoordinates().at(1) > -0.047){
    //   std::cout << "Object is on right side" << std::endl;
    // }
    // else if(openManipulatorTeleop.getPresentArMarkerCoordinates().at(1) > -0.047){
    //   std::cout << "Object is on right side" << std::endl;
    // }
    
    
    usleep(10000);
  
  }



/*  ros::spinOnce();
  openManipulatorTeleop.printText();

  char ch;
  while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    ros::spinOnce();
    openManipulatorTeleop.printText();
    ros::spinOnce();
    openManipulatorTeleop.setGoal(ch);
  }*/

  printf("input : q \tTeleop. is finished\n");
  openManipulatorTeleop.restoreTerminalSettings();

  return 0;
}


//create an absolute value box aorund and throw out any values that don't fit insude the box
//Create hard code edges for the xyz via trial and error\
//Take a moving average of the xyz values. 
/*
MOVING AVERAGE

in order to calculate a moving average we need an array of previous values.
We will need one array for each x,y,&z value.

for 

if moving average is increasing


//lets just mess with the z.

if moving average is increasing,, we need to increase our z axis till we can an AR marker pose of less than 0.35

do we need the moving average.... if ar pose for z is greater than 0.3, increase z position .


while(arpose->z >= 0.3){
  cout << still to far need to move up...
  openmaniplulator z coordinate += 0.1
}


*/