

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

#include "aruco/aruco_nano.h"

#include "perception/marker.h"
#include "perception/stamped_markers.h"

#include "utils.hpp"

cv::Mat camMatrix = (cv::Mat_<double> (3,3) << 817.58251224, 0., 969.25306147, 0., 772.10340966, 655.23802353, 0., 0., 1.);
cv::Mat distCoeff = (cv::Mat_<double> (1,5) << -0.04806606, 1.18213043, -0.01602457, -0.02912741, -1.31690313);

bool finish_recording = false;

void finish_recording_callback(const std_msgs::Bool& msg)
{
  finish_recording = msg.data;
}

int main(int argc, char** argv)
{

  int num_frames = 0;

  char hostname_[HOST_NAME_MAX];
  gethostname(hostname_, HOST_NAME_MAX);
  std::string hostname(hostname_);

  ros::init(argc, argv, hostname + "_aruco_detector");
  ros::NodeHandle nh("~");

  ros::Subscriber sub_srr = nh.subscribe("/perception/stop", 1, finish_recording_callback);
  ros::Publisher pub_marker = nh.advertise<perception::stamped_markers>(hostname + "/markers", 40);
  ros::Publisher pub_frm = nh.advertise<sensor_msgs::Image>(hostname + "/rgb_image", 40);

  auto fourcc = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');

  bool display_video{ false };
  bool use_open_cv{ false };

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
  cap.set(cv::CAP_PROP_BUFFERSIZE, 10);

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
  cv::Mat resized_frm;
  cv::Mat frm_black;

  // ArucoNano variables
  std::vector<aruconano::Marker> markers;

  ros::Time t1 = ros::Time::now();

  perception::stamped_markers markers_msg;

  markers_msg.header.seq = 0;
  markers_msg.header.frame_id = "";  // Not sure what to put here :s
  float markerSize=0.147;//16.5cm and 20.5 with the white border

  while (!finish_recording)
  {
    if (cap.read(frm))
    {
      num_frames++;
      //cv::resize(frm, resized_frm, cv::Size(720, 480), cv::INTER_LINEAR);
      cv::cvtColor(frm, frm_black, cv::COLOR_RGB2GRAY);
      cv::threshold(frm_black, frm_black, 210, 250, cv::THRESH_BINARY);
      markers = aruconano::MarkerDetector::detect(frm_black);

      if (markers.size() > 0)
      {
        markers_msg.header.seq++;
        markers_msg.header.stamp = ros::Time::now();
        markers_msg.markers.clear();

        for (auto e : markers)
        {
          for (auto e : markers)
          {
            if (e.id != 0) continue; // To only have the mushr's tag details
            auto r_t=e.estimatePose(camMatrix,distCoeff,markerSize);
            markers_msg.markers.emplace_back();
            markers_msg.markers.back().id = e.id;
            markers_msg.markers.back().x1 = e[0].x;
            markers_msg.markers.back().y1 = e[0].y;
            markers_msg.markers.back().x2 = e[1].x;
            markers_msg.markers.back().y2 = e[1].y;
            markers_msg.markers.back().x3 = e[2].x;
            markers_msg.markers.back().y3 = e[2].y;
            markers_msg.markers.back().x4 = e[3].x;
            markers_msg.markers.back().y4 = e[3].y;
            markers_msg.markers.back().r1 = r_t.first.at<double>(0,0);
            markers_msg.markers.back().r2 = r_t.first.at<double>(1,0);
            markers_msg.markers.back().r3 = r_t.first.at<double>(2,0);
            markers_msg.markers.back().t1 = r_t.second.at<double>(0,0);
            markers_msg.markers.back().t2 = r_t.second.at<double>(1,0);
            markers_msg.markers.back().t3 = r_t.second.at<double>(2,0);
          }
          //std::cout << markers_msg;
          //if (display_video)
          //{
            //cv::cvtColor(frm_black, frm, cv::COLOR_GRAY2RGB);

            //for (auto e : markers)
            //{
              //e.draw(frm);
            //}
            //cv::imshow("Video", frm);
          //}
        }
        pub_marker.publish(markers_msg);

        auto img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", frm).toImageMsg();
        img_msg->header.stamp = ros::Time::now();
        pub_frm.publish(img_msg);
      }
    }
    ros::spinOnce();
  }

  ros::Time t2 = ros::Time::now();
  std::cout << t2-t1 <<" - " <<num_frames<<std::endl;

  std::cout << "Finished." << std::endl;
  return 0;
}

