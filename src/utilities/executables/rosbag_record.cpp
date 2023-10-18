#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>
#include <rosbag/bag.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>

#include <geometry_msgs/TwistStamped.h>

#include <ackermann_msgs/AckermannDriveStamped.h>

#include <cv_bridge/cv_bridge.h>

#include <XmlRpcValue.h>

#include "utilities/macros.hpp"
#include "utilities/rosbag_record.hpp"
#include "utilities/stop_execution.hpp"

std::atomic<bool> stop = false;
std::string rosbag_directory = "";

utilities::queues_t<std_msgs::String> string_queue;
utilities::queues_t<std_msgs::Int32> int32_queue;
utilities::queues_t<utilities::stamped_markers> stamped_markers_queue;
utilities::queues_t<geometry_msgs::TwistStamped> twist_stamped_queue;
utilities::queues_t<ackermann_msgs::AckermannDriveStamped> ackermann_drive_stamped_queue;
utilities::queues_t<sensor_msgs::Image> image_queue;
utilities::queues_t<sensor_msgs::Imu> imu_queue;

template <typename Queue>
std::size_t process_queue(rosbag::Bag& bag, const Queue& queue)
{
  std::size_t msgs_left{ 0 };
  for (std::size_t idx = 0; idx < queue.size(); ++idx)
  {
    if (!queue[idx]._queue.empty())
    {
      msgs_left += queue[idx]._queue.size();
      auto msg = queue[idx]._queue.front();
      bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
      queue[idx]._queue.pop();
    }
  }
  return msgs_left;
}

template <typename... Queues>
std::size_t process_all_queues(rosbag::Bag& bag, const Queues&... queues)
{
  return (process_queue(bag, queues) + ...);
}

void bag_writter()
{
  rosbag::Bag bag;
  utilities::init_bag(&bag, rosbag_directory);

  ros::Time msg_t;

  std::size_t msgs_left{ 0 };

  while (msgs_left > 0 || !stop)
  {
    msgs_left = process_all_queues(bag, stamped_markers_queue, string_queue, twist_stamped_queue,
                                   ackermann_drive_stamped_queue, image_queue, imu_queue);
    if (stop)
    {
      ROS_INFO_STREAM_ONCE("Remaining messages: " << msgs_left);
    }
  }
  bag.close();
  ROS_INFO_STREAM("Rosbag closed.");
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_record");

  ros::NodeHandle nh("~");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  XmlRpc::XmlRpcValue topics;
  XmlRpc::XmlRpcValue stop_service;
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(topics), topics);
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(stop_service), stop_service);
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(rosbag_directory), rosbag_directory);

  std::vector<ros::Subscriber> subscribers;
  utilities::stop_execution_t stop_execution("ROSBAG_RECORD");

  ros::ServiceServer service =
      nh.advertiseService(stop_service, &utilities::stop_execution_t::callback, &stop_execution);

  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = topics.begin(); it != topics.end(); ++it)
  {
    auto topic_i = topics[it->first];
    const std::string topic_name(topic_i["name"]);
    const std::string topic_type(topic_i["type"]);

    std::cout << "topic_name: " << topic_name << std::endl;
    ROS_INFO_STREAM("Topic: " << topic_name << " - " << topic_type);
    bool registred{ false };
    registred |= string_queue.register_topic(topic_name, topic_type, "string", subscribers, nh);
    registred |= int32_queue.register_topic(topic_name, topic_type, "int32", subscribers, nh);
    registred |= stamped_markers_queue.register_topic(topic_name, topic_type, "stamped_markers", subscribers, nh);
    registred |= twist_stamped_queue.register_topic(topic_name, topic_type, "TwistStamped", subscribers, nh);
    registred |=
        ackermann_drive_stamped_queue.register_topic(topic_name, topic_type, "AckermannDriveStamped", subscribers, nh);
    registred |= image_queue.register_topic(topic_name, topic_type, "sensor_msgs::Image", subscribers, nh);
    registred |= imu_queue.register_topic(topic_name, topic_type, "sensor_msgs::Imu", subscribers, nh);

    if (!registred)
    {
      std::cout << "Unsupported topic '" << topic_name << "' type: " << topic_type << std::endl;
    }
  }

  std::thread thread_b(bag_writter);

  while (!stop)
  {
    stop = stop_execution.status() != utilities::stop_execution_t::Status::running;
    ros::spinOnce();
  }
  ROS_INFO_STREAM("Joining bag writter thread");
  thread_b.join();

  return 0;
}