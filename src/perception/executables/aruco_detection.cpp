#include <algorithm>
#include <chrono>
#include <fstream>
#include <iostream>
#include <limits.h>
#include <signal.h>
#include <thread>
#include <unordered_map>

#include <opencv2/aruco.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>

#include "aruco/aruco_nano.h"

#include "perception/marker.h"
#include "perception/stamped_markers.h"

#include "utils.hpp"

bool finish_recording = false;

void finish_recording_callback(const std_msgs::Bool& msg)
{
  finish_recording = msg.data;
}

int main(int argc, char** argv)
{
  char hostname[HOST_NAME_MAX];
  gethostname(hostname, HOST_NAME_MAX);

  ros::init(argc, argv, std::string(hostname) + "_aruco_detector");
  ros::NodeHandle nh("");

  ros::Subscriber sub_srr = nh.subscribe("/perception/stop", 1, finish_recording_callback);
  ros::Publisher pub_marker = nh.advertise<perception::stamped_markers>(hostname + "/markers", 40);

  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  int height{ 0 };
  int width{ 0 };
  int frequency{ 0 };
  std::string camera_file{ "" };

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(camera_file), camera_file);

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(height), height);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(width), width);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(frequency), frequency);

  cv::VideoCapture cap(camera_index, cv::CAP_V4L2);

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

  cv::Mat frm;
  cv::Mat frm_black;

  std::vector<aruconano::Marker> markers;

  cv::aruco::ArucoDetector detector(dictionary, parameters);
  int marker_id;

  ros::Time t;

  // perception::marker marker;
  perception::stamped_markers markers_msg;

  markers_msg.seq = 0;
  markers_msg.frame_id = "";  // Not sure what to put here :s

  while (!finish_recording)
  {
    if (cap.read(frm))
    {
      cv::cvtColor(frm, frm_black, cv::COLOR_RGB2GRAY);
      cv::threshold(frm_black, frm_black, 210, 250, cv::THRESH_BINARY);
      markers = aruconano::MarkerDetector::detect(frm_black);

      markers_msg.seq++;
      markers_msg.stamp = ros::Time::now();
      markers_msg.markers.clear();

      for (auto e : markers)
      {
        markers_msg.markers.emplace_back(e.id, e[0].x, e[0].y, e[1].x, e[1].y, e[2].x, e[2].y, e[3].x, e[3].y)
      }
      pub_marker.publish(markers_msg);
      if (display_video)
      {
        cv::cvtColor(frm_black, frm, cv::COLOR_GRAY2RGB);

        for (auto e : markers)
        {
          e.draw(frm);
        }
        cv::imshow("Video", frm);
        cv::waitKey(1);
      }
    }
    ros::spinOnce();
  }

  std::cout << "Total Markers: " << total_markers << std::endl;
  std::cout << "Finished." << std::endl;
  return 0;
}