// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from my_interface:srv/StopRobot.idl
// generated code does not contain a copyright notice

#ifndef MY_INTERFACE__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_
#define MY_INTERFACE__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "my_interface/srv/detail/stop_robot__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace my_interface
{

namespace srv
{

namespace builder
{

class Init_StopRobot_Request_stop
{
public:
  Init_StopRobot_Request_stop()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_interface::srv::StopRobot_Request stop(::my_interface::srv::StopRobot_Request::_stop_type arg)
  {
    msg_.stop = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_interface::srv::StopRobot_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interface::srv::StopRobot_Request>()
{
  return my_interface::srv::builder::Init_StopRobot_Request_stop();
}

}  // namespace my_interface


namespace my_interface
{

namespace srv
{

namespace builder
{

class Init_StopRobot_Response_success
{
public:
  Init_StopRobot_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::my_interface::srv::StopRobot_Response success(::my_interface::srv::StopRobot_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::my_interface::srv::StopRobot_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::my_interface::srv::StopRobot_Response>()
{
  return my_interface::srv::builder::Init_StopRobot_Response_success();
}

}  // namespace my_interface

#endif  // MY_INTERFACE__SRV__DETAIL__STOP_ROBOT__BUILDER_HPP_
