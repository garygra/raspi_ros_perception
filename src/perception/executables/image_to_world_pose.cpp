#ifdef IGNORE_THIS_FILE
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
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

#include "aruco/aruco_nano.h"

#include "perception/marker.h"
#include "perception/stamped_markers.h"

#include "utils.hpp"

bool finish_recording = false;

cv::Mat cam_matrix;
cv::Mat dist_coeff;
float markerSize;
perception::stamped_markers current_markers; 
// Detect when the "finish-recording" signal is given
void finish_recording_callback(const std_msgs::Bool& msg)
{
  finish_recording = msg.data;
}

void markers_callback(const perception::stamped_markers& markers_msg)
{
    current_markers = markers_msg; // Might need extra lines to copy the vector
}

// Use cam_matrix, dist_coeff and markerSize to estimate current_pose
void estimate_pose(perception::marker& marker, geometry_msgs::Pose& current_pose)
{
// Debugging: Draw the markers on the image
/**
    for(const auto &m : current_markers.markers)
 *      m.draw(image);
 *    cv::imwrite("/path/to/out.png",image);
 *
 **/
    // Might need to pass marker to opencv::array. There might be a fast way using references...
    // for(const auto &m : current_markers.markers)
    // {
    //   GET R & T FROM cv::solvePnP - https://docs.opencv.org/4.x/d9/d0c/group__calib3d.html#ga549c2075fac14829ff4a58bc931c033d
    // }

    // Now go from R & T to position and quaternion (there is a function in openCV/Eigen to get cuaternion from matrix)
    // (...)
    // current_pose.position.x = (...);
    // current_pose.position.y = (...);
    // current_pose.position.z = (...);

    // current_pose.orientation.x = (...);
    // current_pose.orientation.y = (...);
    // current_pose.orientation.z = (...);
    // current_pose.orientation.w = (...);
}

int main(int argc, char** argv)
{
  char hostname_[HOST_NAME_MAX];
  gethostname(hostname_, HOST_NAME_MAX);
  std::string hostname(hostname_);

  ros::init(argc, argv, hostname + "_image_to_world_pose");
  ros::NodeHandle nh("~");

  ros::Subscriber sub_srr = nh.subscribe("/perception/stop", 1, finish_recording_callback);
  ros::Publisher pub_pose= nh.advertise<geometry_msgs::PoseArray>(hostname + "/markers/world_poses", 40);

    // Given that this are matrices/arrays, we might need to do extra stuff to read from parameters
    // An option could be to have cam_matrix as vectors of vectors and have an extra functions that goes from that to matrix.
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(cam_matrix), cam_matrix);
  perception::get_param_and_check(nh, GET_VARIABLE_NAME(dist_coeff), dist_coeff);

  perception::get_param_and_check(nh, GET_VARIABLE_NAME(markerSize), markerSize);


  ros::Rate loop_rate(frequency * 2);

  // perception::marker marker;
  geometry_msgs::PoseArray pose_array;

  while (!finish_recording)
  {
      if (current_markers.markers.size() > 0)
      {
        pose_array.header = current_markers.header;
        pose_array.header.stamp = ros::Time::now(); // Restamping, this will give an idea of delays
        pose_array.poses.clear();

        for (auto marker : current_markers.markers)
        {
            pose_array.emplace_back();
            estimate_pose(marker, pose_array.back());
        }
        pub_pose.publish(pose_array);
      }
    
    ros::spinOnce();
  }

  std::cout << "Finished." << std::endl;
  return 0;
}
#else
int main() {return 0;}
#endif