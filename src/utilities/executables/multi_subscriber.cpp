#include "ros_type_introspection/ros_introspection.hpp"
#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

using namespace RosIntrospection;

void topicCallback(const topic_tools::ShapeShifter::ConstPtr& msg, const std::string& topic_name,
                   RosIntrospection::Parser& parser)
{
  const std::string& datatype = msg->getDataType();
  const std::string& definition = msg->getMessageDefinition();

  // don't worry if you do this more than once: already registered message are not overwritten.
  parser.registerMessageDefinition(topic_name, RosIntrospection::ROSType(datatype), definition);

  // reuse these opbects to improve efficiency ("static" makes them persistent)
  static std::vector<uint8_t> buffer;
  static std::map<std::string, FlatMessage> flat_containers;
  static std::map<std::string, RenamedValues> renamed_vectors;

  FlatMessage& flat_container = flat_containers[topic_name];
  RenamedValues& renamed_values = renamed_vectors[topic_name];

  // copy raw memory into the buffer
  buffer.resize(msg->size());
  ros::serialization::OStream stream(buffer.data(), buffer.size());
  msg->write(stream);

  // deserialize and rename the vectors
  parser.deserializeIntoFlatContainer(topic_name, Span<uint8_t>(buffer), &flat_container, 100);
  parser.applyNameTransform(topic_name, flat_container, &renamed_values);

  // Print the content of the message
  printf("--------- %s ----------\n", topic_name.c_str());
  for (auto it : renamed_values)
  {
    const std::string& key = it.first;
    const Variant& value = it.second;
    printf(" %s = %f\n", key.c_str(), value.convert<double>());
  }
  for (auto it : flat_container.name)
  {
    const std::string& key = it.first.toStdString();
    const std::string& value = it.second;
    printf(" %s = %s\n", key.c_str(), value.c_str());
  }
}

// usage: pass the name of the file as command line argument
int main(int argc, char** argv)
{
  Parser parser;
  ros::init(argc, argv, "universal_subscriber");
  std::vector<std::string> topic_names = { "/test1" };
  ros::NodeHandle nh;

  // subscribers will be created dynamically, one for each element in topic_names
  std::vector<ros::Subscriber> subscribers;

  for (const std::string& topic_name : topic_names)
  {
    // who is afraid of lambdas and boost::functions ?
    boost::function<void(const topic_tools::ShapeShifter::ConstPtr&)> callback;
    callback = [&parser, topic_name](const topic_tools::ShapeShifter::ConstPtr& msg) -> void {
      topicCallback(msg, topic_name, parser);
    };
    subscribers.push_back(nh.subscribe(topic_name, 10, callback));
  }

  ros::spin();

  return 0;
}
