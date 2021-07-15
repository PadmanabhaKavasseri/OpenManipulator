#include <thread>
#include <unistd.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <float.h>
#include <limits.h>


#include </usr/local/include/librealsense2/rs.h>
#include </usr/local/include/librealsense2/h/rs_pipeline.h>
#include </usr/local/include/librealsense2/h/rs_option.h>
#include </usr/local/include/librealsense2/h/rs_frame.h>


#include "open_manipulator_teleop_keyboard.h"
#include "example.h"

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//                                     These parameters are reconfigurable                                        //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
#define STREAM          RS2_STREAM_DEPTH  // rs2_stream is a types of data provided by RealSense device           //
#define FORMAT          RS2_FORMAT_Z16    // rs2_format is identifies how binary data is encoded within a frame   //
#define WIDTH           640               // Defines the number of columns for each frame or zero for auto resolve//
#define HEIGHT          0                 // Defines the number of lines for each frame or zero for auto resolve  //
#define FPS             30                // Defines the rate of frames per second                                //
#define STREAM_INDEX    0                 // Defines the stream index, used for multiple streams of the same type //
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


std::atomic<bool> done(false); //could make this a memebr variable

OpenManipulatorTeleop::OpenManipulatorTeleop() :node_handle_(""), priv_node_handle_("~"){
  present_joint_angle_.resize(NUM_OF_JOINT);
  present_kinematic_position_.resize(3);

  initClient();
  initSubscriber();
  initCamera();

  //need to set memeber variables here 
  //cam_info first and second accordingly 

  ROS_INFO("OpenManipulator initialization");
}

OpenManipulatorTeleop::~OpenManipulatorTeleop(){
  if(ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
  }
  dltCamera();
}

void OpenManipulatorTeleop::dltCamera(){
  rs2_delete_pipeline_profile(pipeline_profile_);
  rs2_delete_config(config_);
  rs2_delete_pipeline(pipeline_);
  rs2_delete_device(dev_);
  rs2_delete_device_list(device_list_);
  rs2_delete_context(ctx_);
}

void OpenManipulatorTeleop::initClient(){
  goal_joint_space_path_from_present_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path_from_present");
  goal_task_space_path_from_present_position_only_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetKinematicsPose>("goal_task_space_path_from_present_position_only");

  goal_joint_space_path_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_joint_space_path");
  goal_tool_control_client_ = node_handle_.serviceClient<open_manipulator_msgs::SetJointPosition>("goal_tool_control");
}

void OpenManipulatorTeleop::initSubscriber(){
  joint_states_sub_ = node_handle_.subscribe("joint_states", 10, &OpenManipulatorTeleop::jointStatesCallback, this);
  kinematics_pose_sub_ = node_handle_.subscribe("kinematics_pose", 10, &OpenManipulatorTeleop::kinematicsPoseCallback, this);
}

void OpenManipulatorTeleop::initCamera(){
  rs2_error* e = 0;
  e_ = e;

  rs2_context* ctx = rs2_create_context(RS2_API_VERSION, &e);
  check_error(e);
  ctx_ = ctx;

  rs2_device_list* device_list = rs2_query_devices(ctx, &e);
  check_error(e);

  int dev_count = rs2_get_device_count(device_list, &e);
  check_error(e);
  printf("There are %d connected RealSense devices.\n", dev_count);
  if (0 == dev_count)
    exit(EXIT_FAILURE);
      // return EXIT_FAILURE;

  rs2_device* dev = rs2_create_device(device_list, 0, &e);
  check_error(e);
  dev_ = dev;

  print_device_info(dev);


  rs2_pipeline* pipeline =  rs2_create_pipeline(ctx, &e);
  check_error(e);
  pipeline_ = pipeline;

  rs2_config* config = rs2_create_config(&e);
  check_error(e);
  config_ = config;

  // Request a specific configuration
  rs2_config_enable_stream(config, STREAM, STREAM_INDEX, WIDTH, HEIGHT, FORMAT, FPS, &e);
  check_error(e);


  rs2_pipeline_profile* pipeline_profile = rs2_pipeline_start_with_config(pipeline, config, &e);
  pipeline_profile_ = pipeline_profile;
  if (e)
  {
      printf("The connected device doesn't support depth streaming!\n");
      exit(EXIT_FAILURE);
  }
}

void OpenManipulatorTeleop::find_min(rs2_frame* frame, int max_height, int max_width){
  int x,y = 0;
  float min_dist = 10000;
  float curr_dist = 0.0;
  std::vector<double> goalPose;  goalPose.resize(3, 0.0);

  // rs2_depth_frame_get_distance(frame, width / 2, height / 2, &e);

  //could use functional programming here to speed things up

  for(int h = 0; h < max_height; h++){
      for(int w = 0; w < max_width; w++){
          curr_dist = rs2_depth_frame_get_distance(frame,w,h,&e_);
          check_error(e_);

          if(curr_dist < 0.0){
              continue;
          }

          if(curr_dist < min_dist && curr_dist > 0.0){
              min_dist = curr_dist;
              x = w;
              y = h;
          }
      }
  }

  /*
  WIDTH / LEFT||RIGHT
  if width > 384 it is on the west
  if width < 256 it is on the east
  else it is in the middle

  WIDTH / UP||DOWN
  if height < 192 -> is on top
  if height > 288 it is on bottom
  else it is in the middle
  
  printf("input : z \tincrease(++) z axis in task space\n");
    goalPose.at(2) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(ch == 'x' || ch == 'X')
  {
    printf("input : x \tdecrease(--) z axis in task space\n");
    goalPose.at(2) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);




  */
  // if(min_dist > 0.4){
  //   std::cout << "moving forward" << std::endl;
  //   goalPose.resize(3, 0.0);
  //   goalPose.at(0) = DELTA;
  //   setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  // }
  if(min_dist < 0.4 && min_dist > 0.2){
    //move forward
    std::cout << "moving forward" << std::endl;
    goalPose.resize(3, 0.0);
    goalPose.at(0) = DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(min_dist < 0.158){
    //move back
    std::cout << "moving backward" << std::endl;
    goalPose.resize(3, 0.0);
    goalPose.at(0) = -DELTA;
    setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else{
    std::cout << "not moving" << std::endl;
  }


  if(x >=384) {
      printf("\n<- LEFT <-\t");
      //-delta y axes
      goalPose.resize(3, 0.0);
      goalPose.at(1) = -DELTA;
      setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
  }
  else if(x <= 256) {
      printf("\n-> RIGHT ->\t");
      goalPose.resize(3, 0.0);
      goalPose.at(1) = DELTA;
      setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      //+deltta y axis
  }
  else{
      printf("\n| CENTER |\t");
  }

  if (y <= 192){
      printf("\t + UP +\n");
      printf("input : z \tincrease(++) z axis in task space\n");
      //we want to move the arm up increase in z direction
      goalPose.resize(3, 0.0);
      goalPose.at(2) = DELTA;
      setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);

  }
  else if (y >=288){
      printf("\t - DOWN - \n");
      printf("input : x \tdecrease(--) z axis in task space\n");
      goalPose.resize(3, 0.0);
      goalPose.at(2) = -DELTA;
      setTaskSpacePathFromPresentPositionOnly(goalPose, PATH_TIME);
      //we want to move the arm down decrease in z direction
  }
  else{
      printf("\t| CENTER |\n");

  }
  printf("\n+++Min Distance Coordinates   width: %i,   height:%i,   Distance: %f\n", x,y,min_dist);
}

void OpenManipulatorTeleop::getFrames(){

  while (!done)
  {
      // This call waits until a new composite_frame is available
      // composite_frame holds a set of frames. It is used to prevent frame drops
      // The returned object should be released with rs2_release_frame(...)
      rs2_frame* frames = rs2_pipeline_wait_for_frames(pipeline_, RS2_DEFAULT_TIMEOUT, &e_);
      check_error(e_);

      // Returns the number of frames embedded within the composite frame
      int num_of_frames = rs2_embedded_frames_count(frames, &e_);
      // printf("NUM-FRAMES: %i\n", num_of_frames);
      check_error(e_);

      int i;
      for (i = 0; i < num_of_frames; ++i)
      {
          // The retunred object should be released with rs2_release_frame(...)
          rs2_frame* frame = rs2_extract_frame(frames, i, &e_);
          check_error(e_);

          // Check if the given frame can be extended to depth frame interface
          // Accept only depth frames and skip other frames
          if (0 == rs2_is_frame_extendable_to(frame, RS2_EXTENSION_DEPTH_FRAME, &e_))
              continue;

          // Get the depth frame's dimensions
          int width = rs2_get_frame_width(frame, &e_);
          check_error(e_);
          int height = rs2_get_frame_height(frame, &e_);


          // printf("\nHeight: %i, Width: %i\n",height,width);//height = 480 width =640  in my mind height should be the y 
          check_error(e_);

          find_min(frame,height,width);

          //only run this once
          // break;

          // Query the distance from the camera to the object in the center of the image
          float dist_to_center = rs2_depth_frame_get_distance(frame, width / 2, height / 2, &e_);

          //right a method that calls get_distance from start to stop and find the minimum distance in all and output that coordinate and distance

          check_error(e_);

          // Print the distance
          printf("The camera is facing an object %.3f meters away.\n", dist_to_center);

          rs2_release_frame(frame);
      }

      usleep(150000);
      //only want to run this once
      // break;

      rs2_release_frame(frames);
  }
}

void OpenManipulatorTeleop::jointStatesCallback(const sensor_msgs::JointState::ConstPtr &msg){
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

void OpenManipulatorTeleop::kinematicsPoseCallback(const open_manipulator_msgs::KinematicsPose::ConstPtr &msg){
  std::vector<double> temp_position;
  temp_position.push_back(msg->pose.position.x);
  temp_position.push_back(msg->pose.position.y);
  temp_position.push_back(msg->pose.position.z);
  present_kinematic_position_ = temp_position;
}

std::vector<double> OpenManipulatorTeleop::getPresentJointAngle(){
  return present_joint_angle_;
}

std::vector<double> OpenManipulatorTeleop::getPresentKinematicsPose(){
  return present_kinematic_position_;
}

bool OpenManipulatorTeleop::setJointSpacePathFromPresent(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time){
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

bool OpenManipulatorTeleop::setJointSpacePath(std::vector<std::string> joint_name, std::vector<double> joint_angle, double path_time){
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

bool OpenManipulatorTeleop::setTaskSpacePathFromPresentPositionOnly(std::vector<double> kinematics_pose, double path_time){
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

void OpenManipulatorTeleop::printText(){
  while(!done){
    // ros::spinOnce();
    sleep(1);
    // printf("eedeebeedee\n");

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
    printf("---------------------------\n");
  }
}

void OpenManipulatorTeleop::setGoal(char ch){
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
}

void OpenManipulatorTeleop::restoreTerminalSettings(){

    tcsetattr(0, TCSANOW, &oldt_);  /* Apply saved settings */
}

void OpenManipulatorTeleop::disableWaitingForEnter(){
  struct termios newt;

  tcgetattr(0, &oldt_);  /* Save terminal settings */
  newt = oldt_;  /* Init new settings */
  newt.c_lflag &= ~(ICANON | ECHO);  /* Change settings */
  tcsetattr(0, TCSANOW, &newt);  /* Apply settings */
}

void OpenManipulatorTeleop::kbrdController(){
  char ch;
  while (ros::ok() && !done) //the getchar is the blocking line   original -> while (ros::ok() && (ch = std::getchar()) != 'q')
  {
    std::cout << "waiting for input: " << std::endl;
    ch = std::getchar();

    if(ch == 'q'){
      std::cout << "Quitting!" << std::endl;
      done = true;      
    }  
    // ros::spinOnce();
    // openManipulatorTeleop.printText();
    ros::spinOnce();
    setGoal(ch); 
  }
}

int main(int argc, char **argv){

  ros::init(argc, argv, "open_manipulator_teleop");

  OpenManipulatorTeleop openManipulatorTeleop;

  ROS_INFO("OpenManipulator teleoperation using keyboard start");
  openManipulatorTeleop.disableWaitingForEnter();

  std::thread streamFrames(&OpenManipulatorTeleop::getFrames,&openManipulatorTeleop);

  // std::thread printStream(&OpenManipulatorTeleop::printText,&openManipulatorTeleop);

  std::thread getInput(&OpenManipulatorTeleop::kbrdController,&openManipulatorTeleop);
  
  streamFrames.join();
  getInput.join();

  printf("input : q \tTeleop. is finished\n");
  openManipulatorTeleop.restoreTerminalSettings();

  return 0;
}
