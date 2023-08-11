#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <signal.h>
#include <thread>
#include <unordered_map>

#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include <ros/ros.h>
#include <ros/param.h>

#include <std_msgs/Bool.h>

#include <cv_bridge/cv_bridge.h>

#include "utils.hpp"

int main(int argc, char** argv)
{
  char hostname_[HOST_NAME_MAX];
  gethostname(hostname_, HOST_NAME_MAX);
  std::string hostname(hostname_);

  ros::init(argc, argv, hostname + "_framerate_checker");
  ros::NodeHandle nh("~");

  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  int height{ 0 };
  int width{ 0 };
  int frequency{ 0 };
  std::string camera_file{ "" };

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(camera_file), camera_file);

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(height), height);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(width), width);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(frequency), frequency);

  cv::VideoCapture cap(camera_file, cv::CAP_V4L2);

  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, frequency);

  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_WB, 1);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);

  std::cout << "Camera: " << camera_file << std::endl;
  std::cout << "cv::CAP_PROP_FRAME_WIDTH: " << cap.get(cv::CAP_PROP_FRAME_WIDTH) << std::endl;
  std::cout << "cv::CAP_PROP_FRAME_HEIGHT: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT) << std::endl;
  std::cout << "cv::CAP_PROP_FPS: " << cap.get(cv::CAP_PROP_FPS) << std::endl;

  if (!cap.isOpened())
  {
    ROS_ERROR_STREAM("Error opening the video source" << camera_file << ".");
    ROS_ERROR_STREAM("Consider using command 'v4l2-ctl --list-devices'");
    std::cout << "Error opening the video source!" << std::endl;
    std::cout << "Consider using command 'v4l2-ctl --list-devices'" << std::endl;
    exit(-1);
  }
  
  cv::Mat frm;
  const double total_duration{30};
  std::size_t frames_read{0};

  auto start = std::chrono::steady_clock::now();
  auto elapsed_seconds = std::chrono::steady_clock::now() - start;
  while (elapsed_seconds.count() < total_duration) 
  {
    if (cap.read(frm))
    {
        frames_read++;
    }
    elapsed_seconds = std::chrono::steady_clock::now() - start;
  }
  std::cout << "frames_read: " << frames_read << std::endl;
  std::cout << "elapsed_seconds: " << elapsed_seconds.count() << std::endl;
  std::cout << "Finished." << std::endl;
  return 0;
}
