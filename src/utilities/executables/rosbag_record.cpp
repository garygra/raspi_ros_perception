#include <atomic>
#include <chrono>
#include <thread>

#include <ros/ros.h>

#include <rosbag/bag.h>

#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>

#include <sensor_msgs/Image.h>
#include <geometry_msgs/TwistStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <XmlRpcValue.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

#include "rosbag_record.hpp"

std::atomic<bool> finish_recording = false;

std::string rosbag_directory = "";

std::vector<utilities::queued_callback<std_msgs::String>> string_queue;
std::vector<utilities::queued_callback<std_msgs::Int32>> int32_queue;
std::vector<utilities::queued_callback<perception::stamped_markers>> stamped_markers_queue;
std::vector<utilities::queued_callback<geometry_msgs::TwistStamped>> twist_stamped_queue;
std::vector<utilities::queued_callback<ackermann_msgs::AckermannDriveStamped>> ackermann_drive_stamped_queue;

void finish_recording_callback(const std_msgs::Bool& msg)
{
  if (msg.data)
  {
    ROS_INFO_STREAM("Exit command received!");
  }

  finish_recording = msg.data;
}

void init_bag(rosbag::Bag* bag)
{
  std::ostringstream bag_name;
  auto t = std::time(nullptr);
  std::tm tm = *std::localtime(&t);
  bag_name << rosbag_directory << "/bag_" << std::put_time(&tm, "%y%m%d_%H%M") << ".bag";
  bag->open(bag_name.str(), rosbag::bagmode::Write);
  std::cout << "Bag name: " << bag_name.str() << std::endl;
}
void bag_writter()
{
  rosbag::Bag bag;
  init_bag(&bag);

  ros::Time msg_t;

  bool there_are_msgs = true;
  while (there_are_msgs || !finish_recording)
  {
    there_are_msgs = false;
    if (!stamped_markers_queue[0]._queue.empty())
    {
      there_are_msgs = true;
      // stamped_markers_queue[0]._queue_mutex.lock();
      auto msg = stamped_markers_queue[0]._queue.front();
      bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
      stamped_markers_queue[0]._queue.pop();
      // stamped_markers_queue[0]._queue_mutex.unlock();
    }
    if (!string_queue[0]._queue.empty())
    {
      there_are_msgs = true;
      auto msg = string_queue[0]._queue.front();
      bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
      string_queue[0]._queue.pop();
    }
    if (!twist_stamped_queue[0]._queue.empty())
    {
      there_are_msgs = true;
      auto msg = twist_stamped_queue[0]._queue.front();
      bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
      twist_stamped_queue[0]._queue.pop();
    }
    if (!ackermann_drive_stamped_queue[0]._queue.empty())
    {
      there_are_msgs = true;
      auto msg = ackermann_drive_stamped_queue[0]._queue.front();
      bag.write(std::get<0>(msg), std::get<1>(msg), std::get<2>(msg));
      ackermann_drive_stamped_queue[0]._queue.pop();
    }
  }
  bag.close();
}
int main(int argc, char** argv)
{
  ros::init(argc, argv, "rosbag_record");

  ros::NodeHandle nh("~");
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  XmlRpc::XmlRpcValue topics;
  XmlRpc::XmlRpcValue stop_topic;
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(topics), topics);
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(stop_topic), stop_topic);
  utilities::get_param_and_check(nh, GET_VARIABLE_NAME(rosbag_directory), rosbag_directory);

  std::vector<ros::Subscriber> subscribers;

  ros::Subscriber sub_srr = nh.subscribe(stop_topic, 1, finish_recording_callback);

  // std::shared_ptr<rosbag::Bag> bag = std::make_shared<rosbag::Bag>();
  // init_bag(bag.get());

  for (XmlRpc::XmlRpcValue::ValueStruct::const_iterator it = topics.begin(); it != topics.end(); ++it)
  {
    auto topic_i = topics[it->first];
    std::string topic_name(topic_i["name"]);
    std::string topic_type(topic_i["type"]);

    std::cout << "topic_name: " << topic_name << std::endl;
    ROS_INFO_STREAM("Topic: " << topic_name << " - " << topic_type);
    if (topic_type == "string")
    {  // TODO: pass this to a class?
      string_queue.emplace_back(topic_name);
      subscribers.push_back(
          nh.subscribe(topic_name, 100, &utilities::queued_callback<std_msgs::String>::callback, &string_queue.back()));
    }
    else if (topic_type == "int32")
    {  // TODO: pass this to a class?
      int32_queue.emplace_back(topic_name);
      subscribers.push_back(
          nh.subscribe(topic_name, 100, &utilities::queued_callback<std_msgs::Int32>::callback, &int32_queue.back()));
    }
    else if (topic_type == "stamped_markers")
    {  // TODO: pass this to a class?
      stamped_markers_queue.emplace_back(topic_name);
      subscribers.push_back(nh.subscribe(topic_name, 100,
                                         &utilities::queued_callback<perception::stamped_markers>::callback,
                                         &stamped_markers_queue.back()));
    }
    else if (topic_type == "TwistStamped")
    {  // TODO: pass this to a class?
      twist_stamped_queue.emplace_back(topic_name);
      subscribers.push_back(nh.subscribe(topic_name, 100,
                                         &utilities::queued_callback<geometry_msgs::TwistStamped>::callback,
                                         &twist_stamped_queue.back()));
    }
    else if (topic_type == "AckermannDriveStamped")
    {  // TODO: pass this to a class?
      ackermann_drive_stamped_queue.emplace_back(topic_name);
      subscribers.push_back(nh.subscribe(topic_name, 100,
                                         &utilities::queued_callback<ackermann_msgs::AckermannDriveStamped>::callback,
                                         &ackermann_drive_stamped_queue.back()));
    }
    else
    {
      std::cout << "Unsupported topic type!" << std::endl;
    }
  }

  ros::Rate loop_rate(50);

  // std::cout << "Spining..." << std::endl;
  std::thread thread_b(bag_writter);
  while (true)
  {
    ros::spinOnce();
    if (finish_recording)
    {
      thread_b.join();
      break;
    }
  }
  return 0;
}