

#ifndef _CAMERA_D435I_H_
#define _CAMERA_D435I_H_

#include <librealsense2/rs.hpp>
#include <mutex>
#include <stdlib.h>
#include <opencv2/opencv.hpp>

#define IMAGE_WIDTH  640
#define IMAGE_HEIGHT 480

struct CameraIntrinsic
{
   int32_t inputPixelWidth;
   int32_t inputPixelHeight;
   int32_t outputPixelWidth;
   int32_t outputPixelHeight;

   float inputCameraMatrix[9];
   float distortionCoefficient[12];
   float outputCameraMatrix[9];
};


class Camera_D435i
{
public:
   Camera_D435i();
   ~Camera_D435i();

   Camera_D435i( const Camera_D435i & ) = delete;

   bool start();
   bool stop();
   void getImage(cv::Mat & color_m, cv::Mat & gray_m,uint16_t* depth);

   const CameraIntrinsic & getCameraConfiguration( ) const;

private:
   //time stamp
   int64_t realClock;
   int64_t monotonicClock;
   int64_t clockOffset;

   rs2::pipeline pipe;
   rs2::pipeline_profile profiles;
   rs2::config cfg;
   rs2_intrinsics intrinsics;
   CameraIntrinsic configuration;

   rs2::frameset frames;

   //std::mutex lock;
};
#endif
