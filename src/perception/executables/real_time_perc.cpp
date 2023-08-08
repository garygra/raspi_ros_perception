#include "real_time_perception.hpp"
#include <chrono>
#include <ctime> 

#include <sensor_msgs/Image.h>

float max_stationary_dist = 1.1342;

std::time_t start_time;
bool is_first = true;

int count = 0;

void check_freq(sensor_msgs::Image& msg) {
  std::time_t curr = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  if (is_first) {
    start_time = curr;
  }

  double interval_in_secs = difftime(curr, start_time);
  count++;

  std::cout << "Freq: " << count/interval_in_secs << std::endl;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_perc");
    ros::NodeHandle nh("~");
    RealTimePerception rtp;
    // ros::Subscriber sub_odom = nh.subscribe("/car/vesc/odom", 1, &RealTimePerception::odometry_callback, &rtp);
    // ros::Subscriber sub_perc = nh.subscribe("/camera_logitech/pracsys/markers", 1, &RealTimePerception::perception_callback, &rtp);
    ros::Subscriber sub_raspi_perc = nh.subscribe("/camera_publisher/pracsysPi1/rgb_image", 1, &RealTimePerception::raspi_img_callback, &rtp);
    ros::Rate loop_rate(30);
    ros::spin();
}