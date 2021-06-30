#include <cstdint>
#include <vector>
#include <chrono>
#include <iostream>
#include <iostream>
#include <fstream>
#include <opencv2/opencv.hpp>

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <thread>
#include <functional>
#include <sys/sysinfo.h>
#include <time.h>

//ros1
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/String.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>
#include <thread>
#include <unistd.h>
#include <geometry_msgs/Point.h>
#include <sensor_msgs/Image.h>
#include <geometry_msgs/PoseStamped.h>


//ROS2 common headers
/*
#include <image_transport/image_transport.h>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/range.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <cv_bridge/cv_bridge.h>
*/

#include "tensorflow/lite/model.h"
#include "tensorflow/lite/interpreter.h"
#include "tensorflow/lite/kernels/register.h"

#ifdef USE_REALSENSE_D435
#include "Camera_D435i.h"
#endif

using namespace std;
using namespace tflite;


image_transport::Publisher image_pub;
ros::Publisher position_pub;


void publishImg(cv::Mat & m, ros::Time t);
void publishPose(float x, float y, float z, ros::Time t );

int main(int argc, char** argv)
{

	ros::init(argc, argv, "tflite_node");

	ros::NodeHandle n;
	
	ros::Rate loop_rate(10);

	ros::Time t;
	image_transport::ImageTransport it(n);
	image_pub = it.advertise("camera/image", 1);
	
	position_pub = n.advertise<geometry_msgs::PoseStamped>("position", 100);
	
	//ros::Time::init();
	//ros::Time ros_time_base;
	
	//ros::Clock ros_clock(RCL_ROS_TIME);
	
// cv::Mat img=cv::imread("/data/frame0031.jpg");
	//printf("++ test: read image\n");
	
   // cv::imshow("ImageWindow", img);

    //int key = cv::waitKey(0);

  std::string model_file = "../models/detect.tflite";
  std::string label_file = "../models/detect_labels.txt";

  if (argc == 3) {
    model_file = argv[1];
    label_file = argv[2];
  } else if (argc != 1) {
    cout << "usage: " << argv[0] << " [model_file] [label_file]" << std::endl;
    return -1;
  }

  TfLiteStatus status;
  unique_ptr<tflite::FlatBufferModel> model;
  unique_ptr<tflite::Interpreter> interpreter;

  cout << "Loading model: " << model_file << std::endl;
  model = tflite::FlatBufferModel::BuildFromFile(model_file.c_str());
  if (!model) {
	cout << "Failed to load the model." << std::endl;
    return -1;
  }

  tflite::ops::builtin::BuiltinOpResolver resolver;
  if(tflite::InterpreterBuilder(*model, resolver)(&interpreter) != kTfLiteOk)
  {
      std::cout << "Failed to build Interpreter." << std::endl;
	  return -1;
  }

  status = interpreter->AllocateTensors();
  if (status != kTfLiteOk) {
    std::cout << "Failed to allocate the memory for tensors." << std::endl;
    return -1;
  }

  //check model info
  int input_idx, input_width, input_height, input_channels, input_data_type;

  cout << "input tensor num:" << interpreter->inputs().size() << std::endl;
  if(interpreter->inputs().size() > 1)
  {
	  cout << "error: not support input tensor num > 1"  << std::endl;
	  return -1;
  }
  else
  {
     input_idx = interpreter->inputs()[0];

     TfLiteIntArray* input_dims = interpreter->tensor(input_idx)->dims;
     cout << "input tensor name: " << interpreter->GetInputName(0) << ", dimension:"  << input_dims->size  << std::endl;
     input_width = input_dims->data[1];
     input_height = input_dims->data[2];
     input_channels = input_dims->data[3];
     input_data_type = interpreter->tensor(input_idx)->type;
     printf("+++++++input image, input_width=%d, input_height=%d\n", input_width, input_height );
     cout << "input shape: " << input_dims->data[0];
     for(int j = 1; j< input_dims->size; j++)
     {
         cout << "X" << input_dims->data[j];
     }
     cout << std::endl;

     cout << "input type:"  << input_data_type << std::endl; //defined in common.h, see enum TfLiteType, kTfLiteFloat32: 1/ kTfLiteUInt8: 3
  }

  cout << "output tensor num:" << interpreter->outputs().size() << std::endl;
  if(interpreter->outputs().size() > 0)
  {
	  int output_idx, output_type;
	  for(int i = 0; i < interpreter->outputs().size(); ++i)
	  {
	      output_idx = interpreter->outputs()[i];
	      TfLiteIntArray* output_dims = interpreter->tensor(output_idx)->dims;
	      cout << "output tensor[" << i << "], name: " << interpreter->GetOutputName(i) << ", dimension:"  << output_dims->size  << std::endl;

	      cout << "output shape: " << output_dims->data[0];
	      for(int j = 1; j< output_dims->size; j++)
	      {
	          cout << "X" << output_dims->data[j];
	      }
	      cout << std::endl;

	      output_type = interpreter->tensor(output_idx)->type;
	      cout << "output type:"  << output_type << std::endl; //defined in common.h, see enum TfLiteType, kTfLiteFloat32: 1/ kTfLiteUInt8: 3
	  }
  }

  if(interpreter->outputs().size() != 1 && interpreter->outputs().size() != 4)
  {
	  cout << "error: unsupported output tensor num."  << std::endl;
	  return -1;
  }

#ifdef USE_REALSENSE_D435

  Camera_D435i cam;
  cam.start();

  uint16_t depth[IMAGE_WIDTH * IMAGE_HEIGHT];

#else
  cv::VideoCapture cam(0); //check if /dev/video0 exists
  //cv::VideoCapture cam(../xxx.mp4); //from a video file

  if (!cam.isOpened()) {
    std::cerr << "Failed to open camera." << std::endl;
    return -1;
  }
#endif

  std::cout << "Loading labels: " << label_file << std::endl;
  std::ifstream file(label_file);
  if (!file) {
    std::cerr << "Failed to read " << label_file << "." << std::endl;
    return -1;
  }

  std::vector<std::string> labels;
  std::string line;
  while (std::getline(file, line))
    labels.push_back(line);

  uint8_t *data_int8 = nullptr;
  float *data_f32 = nullptr;

  //get input tensor raw data pointer, will copy data here
  if (input_data_type == kTfLiteFloat32)
	  data_f32 = interpreter->typed_tensor<float>(input_idx);
  else if (input_data_type == kTfLiteUInt8)
	  data_int8 = interpreter->typed_tensor<uint8_t>(input_idx);


  interpreter->SetNumThreads(4);
  interpreter->UseNNAPI(0);
  
  
	cv::Mat frame, frame_gray;
  while (n.ok())
  {
    
#ifdef USE_REALSENSE_D435
    cam.getImage(frame, frame_gray, depth);
	CameraIntrinsic configuration = cam.getCameraConfiguration();
//    cv::imshow("ImageWindow", frame);

#else
    cam >> frame;
#endif

   //int key = cv::waitKey(1);
   //if (key == 32) // KEY SPACE
     //  break;
// cv::imshow("window", frame);
    cv::Mat resized(input_height, input_width, frame.type());
    cv::resize(frame, resized, resized.size());
	// cv::imshow("window1", resized);

    if (input_data_type == kTfLiteFloat32)
        memcpy(data_f32, resized.data, resized.total() * resized.elemSize());
    else if (input_data_type == kTfLiteUInt8)
        memcpy(data_int8, resized.data, resized.total() * resized.elemSize());

    status = interpreter->Invoke();
    if (status != kTfLiteOk) {
        //cv::imshow("window", frame);
        continue;
    }

    if(interpreter->outputs().size() == 1)  //image classification
    {
		
    	int output_idx, output_size, output_type;
         //printf("+++ready to image classify \n");
  		output_idx = interpreter->outputs()[0];
  		TfLiteIntArray* output_dims = interpreter->tensor(output_idx)->dims;
  	    output_size = output_dims->data[output_dims->size - 1];
  	    output_type = interpreter->tensor(output_idx)->type;

        std::vector<std::pair<float, int>> results;
        if (output_type == kTfLiteFloat32) {
            float *scores = interpreter->typed_output_tensor<float>(0);
            for (int i = 0; i < output_size; ++i) {
                float value = (scores[i] - 127) / 127.0;
                if (value < 0.1)
                    continue;
                results.push_back(std::pair<float, int>(value, i));
            }
        } else if (output_type == kTfLiteUInt8) {
            uint8_t *scores = interpreter->typed_output_tensor<uint8_t>(0);
            for (int i = 0; i < output_size; ++i) {
                float value = (float)scores[i] / 255.0;
                if (value < 0.2)
                    continue;
                results.push_back(std::pair<float, int>(value, i));
            }
        }

        std::sort(results.begin(), results.end(),
            [](std::pair<float, int>& x, std::pair<float, int>& y) -> int { return x.first > y.first; });

        int n = 0;
        for (const auto& result : results) {
            std::stringstream ss;
            ss << result.first << ": " << labels[result.second];
            cv::putText(frame_gray, ss.str(), cv::Point(50, 50 + 30 * n), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0));
            if (++n >= 3) break;
        }

        //show on original frame
       // cv::imshow("window", frame);
    }
    else if(interpreter->outputs().size() == 4) //object detection
    {
        float *locations = interpreter->typed_output_tensor<float>(0);  // 10X4, ymin, xin, ymax, xmax
        float *cls = interpreter->typed_output_tensor<float>(1);       //10
        float *scores = interpreter->typed_output_tensor<float>(2);    //10

        int n = 0;
		float X, Y, Z;
		//printf("+++ready to detct object\n");
        for(int i = 0; i < 10; i++)
        {
        	std::stringstream ss;
        	cv::Rect rec;
        	if( scores[i] > 0.4 )
        	{
        		int label_idx = (int)cls[i];
        		ss  << scores[i] << ": " << labels[label_idx + 1];

        		auto ymin   = locations[4*i] * frame.rows;
        		auto xmin   = locations[4*i+1]* frame.cols;
        		auto ymax   = locations[4*i+2]* frame.rows;
        		auto xmax   = locations[4*i+3]* frame.cols;
        		auto width  = xmax - xmin;
        		auto height = ymax - ymin;

        		rec.x = xmin;
        	    rec.y = ymin;
        		rec.width = width;
        		rec.height = height;
        		cv::rectangle(frame_gray, rec, cv::Scalar(0,255,0), 1);
        		cv::putText(frame_gray, ss.str(), cv::Point(rec.x+3, rec.y+15), cv::FONT_HERSHEY_COMPLEX, 0.5, cv::Scalar(0,255,0));
			    int idx = (ymin+height/2)*frame.cols + xmin+width/2;
				Z = depth[idx];
				X = (xmin + width/2 - configuration.inputCameraMatrix[2])*Z/configuration.inputCameraMatrix[0];
				Y = (ymin + height/2 - configuration.inputCameraMatrix[5])*Z/configuration.inputCameraMatrix[4];
			    printf("+++++bounding box  score=%f, x=%d, y=%d, realpose X=%f, Y=%f, Z=%f\n", scores[i], rec.x, rec.y, X,Y, Z);
                cout << "ss.str(): " << ss.str() << endl;
        	    if (++n >= 1) break;
        	}
        }
        //printf("+++ready to draw image\n");
        //cv::imshow("ImageWindow", frame);
		
		//ros_time_base = ros_clock.now();
		t = ros::Time::now();		
		publishImg(frame_gray, t);
		publishPose(X/1000, Y/1000, Z/1000, t);

	//	int key = cv::waitKey(1);

    }
	ros::spinOnce();
	loop_rate.sleep();

#if 0
    if (status == kTfLiteOk && n > 0) {
       int i = results[0].second;
       //printf("index %d, %s\n", i, labels[i].c_str());
       std::string saved_file = labels[i] + ".jpg";
      // cv::imwrite(saved_file, frame);
    }
#endif


  }

  cv::destroyAllWindows();

#ifdef USE_REALSENSE_D435
  cam.stop();
#endif



 image_pub.shutdown();
 ros::shutdown();

 

  return 0;
}


void publishImg(cv::Mat & m, ros::Time t)
{
    sensor_msgs::ImagePtr img;
    img = cv_bridge::CvImage(
      std_msgs::Header(), sensor_msgs::image_encodings::MONO8, m).toImageMsg();
	  
    img->width = IMAGE_WIDTH;
    img->height = IMAGE_HEIGHT;
    img->is_bigendian = false;
    img->step = IMAGE_WIDTH;
    img->header.frame_id = "image_frame";
    //img->header.stamp = t;
    image_pub.publish(img);
}

void publishPose(float x, float y, float z, ros::Time t )
{
	geometry_msgs::PoseStamped output_msg;
	
    output_msg.pose.position.x = x;
    output_msg.pose.position.y = y;
    output_msg.pose.position.z = z;
	output_msg.header.frame_id = "position";
    //output_msg.header.stamp = t;
    position_pub.publish(output_msg); 


}


