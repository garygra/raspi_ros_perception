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

bool finish_recording = false;

void finish_recording_callback(const std_msgs::Bool& msg)
{
  finish_recording = msg.data;
}

int main(int argc, char** argv)
{
  char hostname_[HOST_NAME_MAX];
  gethostname(hostname_, HOST_NAME_MAX);
  std::string hostname(hostname_);

  ros::init(argc, argv, hostname + "_aruco_detector");
  ros::NodeHandle nh("~");

  ros::Subscriber sub_srr = nh.subscribe("/perception/stop", 1, finish_recording_callback);
  ros::Publisher pub_frm =
      nh.advertise<sensor_msgs::Image>(hostname + "/rgb_image", 40);

  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  bool display_video{ false };

  int height{ 0 };
  int width{ 0 };
  int frequency{ 0 };
  std::string camera_file{ "" };

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(camera_file), camera_file);

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(height), height);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(width), width);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(frequency), frequency);

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(display_video), display_video);

  cv::VideoCapture cap(camera_file, cv::CAP_V4L2);

  cap.set(cv::CAP_PROP_FRAME_WIDTH, width);
  cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
  cap.set(cv::CAP_PROP_FPS, frequency);

  cap.set(cv::CAP_PROP_AUTOFOCUS, 0);
  cap.set(cv::CAP_PROP_AUTO_WB, 1);
  cap.set(cv::CAP_PROP_AUTO_EXPOSURE, 0);

  ROS_DEBUG_STREAM("Camera: " << camera_file);
  ROS_DEBUG_STREAM("cv::CAP_PROP_FRAME_WIDTH: " << cap.get(cv::CAP_PROP_FRAME_WIDTH));
  ROS_DEBUG_STREAM("cv::CAP_PROP_FRAME_HEIGHT: " << cap.get(cv::CAP_PROP_FRAME_HEIGHT));
  ROS_DEBUG_STREAM("cv::CAP_PROP_FPS: " << cap.get(cv::CAP_PROP_FPS));

  if (!cap.isOpened())
  {
    ROS_ERROR_STREAM("Error opening the video source" << camera_file << ".");
    ROS_ERROR_STREAM("Consider using command 'v4l2-ctl --list-devices'");
    std::cout << "Error opening the video source!" << std::endl;
    std::cout << "Consider using command 'v4l2-ctl --list-devices'" << std::endl;
    exit(-1);
  }
  ros::Rate loop_rate(frequency * 2);

  ros::Time t;
  cv::Mat frm;

  while (!finish_recording)
  {
    if (cap.read(frm))
    {
      cv::resize(frm, frm, cv::Size(720, 480), cv::INTER_LINEAR);

      auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frm).toImageMsg();
      img_msg->header.stamp = ros::Time::now();
      pub_frm.publish(img_msg);
    }
    ros::spinOnce();
  }

  std::cout << "Finished." << std::endl;
  return 0;
}
