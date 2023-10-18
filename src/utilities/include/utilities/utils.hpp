#define GET_VARIABLE_NAME(Variable) (#Variable)

#define DBG printf("%s: %d\n", __PRETTY_FUNCTION__, __LINE__);

namespace utilities
{
template <typename T>
void get_param_and_check(ros::NodeHandle& nh, const std::string& var_name, T& var)
{
  if (!nh.getParam(var_name, var))
  {
    ROS_FATAL_STREAM(var_name << " parameter is needed.");
    exit(-1);
  }
}

template <typename T>
void print_container(const std::string& name, const T& container)
{
  std::cout << name << ": ";
  for (auto e : container)
  {
    std::cout << e << ", ";
  }
  std::cout << std::endl;
}
}  // namespace utilities