#pragma once
#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <chrono>

#include <nav_msgs/Odometry.h>
#include "perception/stamped_markers.h"
#include "perception/marker.h"

#include <torch/torch.h>
#include <torch/script.h>

class RealTimePerception 
{
    private:
    float odom_x, odom_y, odom_z, init_odom_x, init_odom_y, init_odom_z, perc_x, perc_y, init_perc_x, init_perc_y;
    bool first_perception_message, first_odometry_message;
    int perception_callback_count = 0;
    double init_timestamp, last_log_timestamp;
    std::vector<double> log_times;

    public:
    std::vector<double> current_estimate;
    std::string controller_path = ros::package::getPath("perception")+"/networks/MushrObs.pt";

    torch::jit::script::Module controller;
    std::vector<torch::jit::IValue> controller_inputs;

    public:
    RealTimePerception()
    {
        first_perception_message = true;
        first_odometry_message = true;
        
        ROS_INFO_STREAM(controller_path);
        torch::manual_seed(123456);
		torch::NoGradGuard no_grad;
        torch::Device device(torch::kCPU);
        
        try
        {
            controller = torch::jit::load(controller_path,device);
            ROS_INFO_STREAM("Loaded the controller.");
        }
        catch(const c10::Error& e)
        {
            ROS_ERROR_STREAM("Error loading the model\n");
        }
    };
    ~RealTimePerception() 
    {
    };

    void odometry_callback(const nav_msgs::Odometry& msg)
    {
        odom_x = msg.pose.pose.position.x;
        odom_y = msg.pose.pose.position.y;
        odom_z = msg.pose.pose.position.z;

        if (first_odometry_message) 
        {
            init_odom_x = odom_x;
            init_odom_y = odom_y;
            init_odom_z = odom_z;
            first_odometry_message = false;
        }

        float dist = std::sqrt(
            std::pow(init_odom_x - odom_x, 2) + 
            std::pow(init_odom_y - odom_y, 2) + 
            std::pow(init_odom_z - odom_z, 2)
        );
    }

    void perception_callback(const perception::stamped_markers& msg)
    {
        perception_callback_count++;
        double timestamp = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        if (first_perception_message) {
            init_timestamp = timestamp;
            last_log_timestamp = timestamp;
            first_perception_message = false;
        }

        double log_time_interval = timestamp - last_log_timestamp;
        // std::cout << timestamp << " " << last_log_timestamp << " " << log_time << std::endl;
        if (log_time_interval > 10) {
            last_log_timestamp = timestamp;
            double duration_in_secs = (double)(timestamp - init_timestamp);
            std::cout <<
                "Peception Message Count: " <<
                perception_callback_count << 
                " | Time duration: " << 
                duration_in_secs<<
                " | Throughput (msgs/sec): " <<
                perception_callback_count/duration_in_secs <<
                std::endl;
        }

        // std::vector<perception::marker> markers = msg.markers;
        // for(perception::marker mrkr : markers) 
        // {
        //     if (mrkr.id != 0) continue;
        //     float x1 = mrkr.x1, x2 = mrkr.x2, x3 = mrkr.x3, x4 = mrkr.x4;
        //     float y1 = mrkr.y1, y2 = mrkr.y2, y3 = mrkr.y3, y4 = mrkr.y4;

        //     perc_x = (x1 + x2 + x3 + x4)/4;
        //     perc_y = (y1 + y2 + y3 + y4)/4;

        //     if (first_perception_message) 
        //     {
        //         init_perc_x = perc_x;
        //         init_perc_y = perc_y;
        //         first_perception_message = false;
        //     }

        //     float dist = std::sqrt(std::pow(init_perc_x - perc_x, 2) + std::pow(init_perc_y - perc_y, 2));
        // }
    }
};