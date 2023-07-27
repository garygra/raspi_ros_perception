#include "real_time_perception.hpp"

float max_stationary_dist = 1.1342;

int main(int argc, char** argv) {
    ros::init(argc, argv, "real_time_perc");
    ros::NodeHandle nh("~");
    RealTimePerception rtp;
    ros::Subscriber sub_odom = nh.subscribe("/car/vesc/odom", 1, &RealTimePerception::odometry_callback, &rtp);
    ros::Subscriber sub_perc = nh.subscribe("/camera_logitech/pracsys/markers", 1, &RealTimePerception::perception_callback, &rtp);
    ros::Rate loop_rate(10);
    ros::spin();
}