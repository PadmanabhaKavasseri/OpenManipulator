#ifndef OPEN_MANIPULATOR_TELEOP_H
#define OPEN_MANIPULATOR_TELEOP_H

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>

#include <termios.h>
#include "open_manipulator_msgs/SetJointPosition.h"
#include "open_manipulator_msgs/SetKinematicsPose.h"



#define NUM_OF_JOINT 4
#define DELTA 0.01
#define JOINT_DELTA 0.05
#define PATH_TIME 0.5

class OpenManipulatorTeleop
{
 private:
  // ROS NodeHandle
  ros::NodeHandle node_handle_;
  ros::NodeHandle priv_node_handle_;

  ros::ServiceClient goal_joint_space_path_from_present_client_;
  ros::ServiceClient goal_task_space_path_from_present_position_only_client_;
  ros::ServiceClient goal_joint_space_path_client_;
  ros::ServiceClient goal_tool_control_client_;

  ros::Subscriber joint_states_sub_;
  ros::Subscriber kinematics_pose_sub_;

  std::vector<double> present_joint_angle_;
  std::vector<double> present_kinematic_position_;

  struct termios oldt_;

  rs2_error* e_;
  rs2_pipeline* pipeline_;
  rs2_pipeline_profile* pipeline_profile_;
  rs2_config* config_;
  rs2_device* dev_;
  rs2_device_list* device_list_;
  rs2_context* ctx_;




 public:

  OpenManipulatorTeleop();
  ~OpenManipulatorTeleop();

  
  void initClient();
  void initSubscriber();

  void jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg);
  void kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg);

  std::vector<double> getPresentJointAngle();
  std::vector<double> getPresentKinematicsPose();

  bool setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time);
  bool setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time);

  bool setToolControl(std::vector<double> joint_angle);

  void printText();
  void kbrdController();
  void setGoal(char ch);

  void restoreTerminalSettings(void);
  void disableWaitingForEnter(void);

  void find_min(rs2_frame* frame, int max_height, int max_width);

  void initCamera();
  void getFrames();
  void dltCamera();



};

#endif //OPEN_MANIPULATOR_TELEOP_H


