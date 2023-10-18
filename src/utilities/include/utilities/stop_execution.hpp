#include "ros/ros.h"
#include "utilities/stop_execution.h"

namespace utilities
{
struct stop_execution_t
{
  // Status of the node: either running, message receieved to stop exec
  enum Status
  {
    running,
    stop_execution,
  };

  stop_execution_t(const std::string msg) : _status(Status::running), _msg(msg)
  {
  }

  inline Status status() const
  {
    return _status;
  }

  void status(const Status status)
  {
    _status = status;
  }

  // Convinient method to do: `while(stop_execution.continue_running()){(...)}
  inline bool continue_running() const
  {
    return _status != utilities::stop_execution_t::Status::running;
  }

  bool callback(utilities::stop_execution::Request& req, utilities::stop_execution::Response& res)
  {
    if (req.stop)
    {
      _status = Status::stop_execution;
      ROS_INFO_STREAM("[" << _msg << "]: Stopping execution");
    }
    return true;
  }

  Status _status;
  const std::string _msg;
};
}  // namespace utilities