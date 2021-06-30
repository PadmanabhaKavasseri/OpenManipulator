
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <inttypes.h>
#include <thread>
#include <functional>

#include "Camera_D435i.h"

using namespace cv;
using namespace std;


Camera_D435i::Camera_D435i()
{
   clockOffset = 0;

}

Camera_D435i::~Camera_D435i()
{
}


void Camera_D435i::getImage(cv::Mat & color_m, cv::Mat & gray_m,  uint16_t* depth)
{

   rs2::align align(RS2_STREAM_COLOR);

   frames = pipe.wait_for_frames();
   frames = align.process(frames);
    
   //Get each frame
   rs2::frame color_frame = frames.get_color_frame();
   rs2::depth_frame depth_frame = frames.get_depth_frame();
   
   memcpy((void *)depth, depth_frame.get_data(), IMAGE_WIDTH*IMAGE_HEIGHT*2 );

   //float dist_to_center = depth_frame.get_distance(IMAGE_WIDTH / 2, IMAGE_HEIGHT / 2);
   //std::cout << "The camera is facing an object " << dist_to_center << " meters away " << std::endl;

#if 1
   Mat color_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC3, (void*)color_frame.get_data(), Mat::AUTO_STEP);
   //cv::imshow(window_color, color_image);
     color_image.copyTo(color_m);
   
  
   Mat gray_image(Size(IMAGE_WIDTH, IMAGE_HEIGHT), CV_8UC1);
   unsigned char *pCData=(unsigned char*)color_frame.get_data();
   
      unsigned int v;
      for(int i=0; i<IMAGE_WIDTH*IMAGE_HEIGHT; ++i)
      {
         v = pCData[3*i] * 299 + pCData[3*i + 1] * 587 + pCData[3*i + 2] * 114 + 500;      
         v = v / 1000;
         gray_image.data[i] = (unsigned char)v;
      }
	  gray_image.copyTo(gray_m);
     // cv::imshow("window_color", color_image);
      //cv::imshow("window", gray_image);
   //////////////////////
 #endif  
   

   //printf("++++++covert realsense image from rgb to gray\n");



}


bool Camera_D435i::start()
{
   //config stream
   cfg.enable_stream(RS2_STREAM_COLOR, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_BGR8, 15);
   cfg.enable_stream(RS2_STREAM_DEPTH, IMAGE_WIDTH, IMAGE_HEIGHT, RS2_FORMAT_Z16, 15);

   //enable config
   profiles = pipe.start(cfg);
   
   rs2::stream_profile color_profile = profiles.get_stream(RS2_STREAM_COLOR);
   if (auto video_profile = color_profile.as<rs2::video_stream_profile>())
   {
      try {
         intrinsics = video_profile.get_intrinsics();
                  
		 printf("intrinsics: ppx %.4f, ppy %.4f, fx %.4f, fy %.4f, model %d\n",
			  intrinsics.ppx, intrinsics.ppy,
			  intrinsics.fx, intrinsics.fy, intrinsics.model);
		 printf("intrinsics coeffs: %.4f, %.4f, %.4f, %.4f, %.4f\n",
              intrinsics.coeffs[0], intrinsics.coeffs[1], intrinsics.coeffs[2],
              intrinsics.coeffs[3], intrinsics.coeffs[4]);
              configuration.inputPixelWidth = intrinsics.width;
              configuration.inputPixelHeight = intrinsics.height;
	     configuration.outputPixelWidth = intrinsics.width;
	     configuration.outputPixelHeight = intrinsics.height;

         memset(configuration.inputCameraMatrix, 0, sizeof(configuration.inputCameraMatrix));
         configuration.inputCameraMatrix[0] = intrinsics.fx;
         configuration.inputCameraMatrix[2] = intrinsics.ppx;
         configuration.inputCameraMatrix[4] = intrinsics.fy;
         configuration.inputCameraMatrix[5] = intrinsics.ppy;
         configuration.inputCameraMatrix[8] = 1.f;
         memcpy(configuration.outputCameraMatrix, configuration.inputCameraMatrix, sizeof(configuration.inputCameraMatrix));

         memset( configuration.distortionCoefficient, 0, sizeof( configuration.distortionCoefficient ) );
         configuration.distortionCoefficient[0] = intrinsics.coeffs[0];
         configuration.distortionCoefficient[1] = intrinsics.coeffs[1];
         configuration.distortionCoefficient[2] = intrinsics.coeffs[2];
         configuration.distortionCoefficient[3] = intrinsics.coeffs[3];
         configuration.distortionCoefficient[4] = intrinsics.coeffs[4];
	  }
	  catch (const std::exception& e) {
	     printf("failed to get camera intrinsic\n");
	  }
   }

   return true;
}


bool Camera_D435i::stop()
{
   pipe.stop();

   return true;
}


const CameraIntrinsic & Camera_D435i::getCameraConfiguration( ) const
{
   return configuration;
}
