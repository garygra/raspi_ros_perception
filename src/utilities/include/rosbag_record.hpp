#pragma once
#include <any>

#include "perception/stamped_markers.h"
#include "utilities/omnirobot.h"
#include "utils.hpp"
namespace utilities
{
template <typename Msg>
struct queued_callback
{
  queued_callback() = delete;
  queued_callback(const std::string topic_name) : _topic_name(topic_name)
  {
    std::cout << "topic_name: " << _topic_name << std::endl;
  };
  static inline std::mutex _queue_mutex;
  static inline std::queue<std::tuple<std::string, ros::Time, typename Msg::ConstPtr>> _queue;

  // void callback(typename Msg::ConstPtr msg, const std::string topic)
  void callback(const ros::MessageEvent<Msg const>& event)
  {
    // const ros::M_string& header = event.getConnectionHeader();
    std::string topic = event.getConnectionHeader().at("topic");

    // const std_msgs::StringConstPtr& msg = event.getMessage();
    // std::cout << "topic_name: " << topic << " " << _queue.size() << std::endl;

    _queue.push(std::make_tuple(topic, ros::Time::now(), event.getMessage()));
    // _queue_mutex.unlock();
  }

private:
  std::string _topic_name;
};

}  // namespace utilities
