// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_interfaces:action/FollowPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "action_interfaces/action/follow_path.hpp"


#ifndef ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
#define ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_interfaces/action/detail/follow_path__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_Goal_path
{
public:
  Init_FollowPath_Goal_path()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::FollowPath_Goal path(::action_interfaces::action::FollowPath_Goal::_path_type arg)
  {
    msg_.path = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_Goal>()
{
  return action_interfaces::action::builder::Init_FollowPath_Goal_path();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_Result_message
{
public:
  explicit Init_FollowPath_Result_message(::action_interfaces::action::FollowPath_Result & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_Result message(::action_interfaces::action::FollowPath_Result::_message_type arg)
  {
    msg_.message = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Result msg_;
};

class Init_FollowPath_Result_success
{
public:
  Init_FollowPath_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Result_message success(::action_interfaces::action::FollowPath_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return Init_FollowPath_Result_message(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_Result>()
{
  return action_interfaces::action::builder::Init_FollowPath_Result_success();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_Feedback_distance_to_goal
{
public:
  explicit Init_FollowPath_Feedback_distance_to_goal(::action_interfaces::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_Feedback distance_to_goal(::action_interfaces::action::FollowPath_Feedback::_distance_to_goal_type arg)
  {
    msg_.distance_to_goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_total_waypoints
{
public:
  explicit Init_FollowPath_Feedback_total_waypoints(::action_interfaces::action::FollowPath_Feedback & msg)
  : msg_(msg)
  {}
  Init_FollowPath_Feedback_distance_to_goal total_waypoints(::action_interfaces::action::FollowPath_Feedback::_total_waypoints_type arg)
  {
    msg_.total_waypoints = std::move(arg);
    return Init_FollowPath_Feedback_distance_to_goal(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Feedback msg_;
};

class Init_FollowPath_Feedback_current_waypoint
{
public:
  Init_FollowPath_Feedback_current_waypoint()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_Feedback_total_waypoints current_waypoint(::action_interfaces::action::FollowPath_Feedback::_current_waypoint_type arg)
  {
    msg_.current_waypoint = std::move(arg);
    return Init_FollowPath_Feedback_total_waypoints(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_Feedback>()
{
  return action_interfaces::action::builder::Init_FollowPath_Feedback_current_waypoint();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Request_goal
{
public:
  explicit Init_FollowPath_SendGoal_Request_goal(::action_interfaces::action::FollowPath_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_SendGoal_Request goal(::action_interfaces::action::FollowPath_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Request msg_;
};

class Init_FollowPath_SendGoal_Request_goal_id
{
public:
  Init_FollowPath_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Request_goal goal_id(::action_interfaces::action::FollowPath_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_SendGoal_Request_goal(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_SendGoal_Request>()
{
  return action_interfaces::action::builder::Init_FollowPath_SendGoal_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Response_stamp
{
public:
  explicit Init_FollowPath_SendGoal_Response_stamp(::action_interfaces::action::FollowPath_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_SendGoal_Response stamp(::action_interfaces::action::FollowPath_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Response msg_;
};

class Init_FollowPath_SendGoal_Response_accepted
{
public:
  Init_FollowPath_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Response_stamp accepted(::action_interfaces::action::FollowPath_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_FollowPath_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_SendGoal_Response>()
{
  return action_interfaces::action::builder::Init_FollowPath_SendGoal_Response_accepted();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_SendGoal_Event_response
{
public:
  explicit Init_FollowPath_SendGoal_Event_response(::action_interfaces::action::FollowPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_SendGoal_Event response(::action_interfaces::action::FollowPath_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Event msg_;
};

class Init_FollowPath_SendGoal_Event_request
{
public:
  explicit Init_FollowPath_SendGoal_Event_request(::action_interfaces::action::FollowPath_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_FollowPath_SendGoal_Event_response request(::action_interfaces::action::FollowPath_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FollowPath_SendGoal_Event_response(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Event msg_;
};

class Init_FollowPath_SendGoal_Event_info
{
public:
  Init_FollowPath_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_SendGoal_Event_request info(::action_interfaces::action::FollowPath_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FollowPath_SendGoal_Event_request(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_SendGoal_Event>()
{
  return action_interfaces::action::builder::Init_FollowPath_SendGoal_Event_info();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Request_goal_id
{
public:
  Init_FollowPath_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::FollowPath_GetResult_Request goal_id(::action_interfaces::action::FollowPath_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_GetResult_Request>()
{
  return action_interfaces::action::builder::Init_FollowPath_GetResult_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Response_result
{
public:
  explicit Init_FollowPath_GetResult_Response_result(::action_interfaces::action::FollowPath_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_GetResult_Response result(::action_interfaces::action::FollowPath_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Response msg_;
};

class Init_FollowPath_GetResult_Response_status
{
public:
  Init_FollowPath_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_GetResult_Response_result status(::action_interfaces::action::FollowPath_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_FollowPath_GetResult_Response_result(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_GetResult_Response>()
{
  return action_interfaces::action::builder::Init_FollowPath_GetResult_Response_status();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_GetResult_Event_response
{
public:
  explicit Init_FollowPath_GetResult_Event_response(::action_interfaces::action::FollowPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_GetResult_Event response(::action_interfaces::action::FollowPath_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Event msg_;
};

class Init_FollowPath_GetResult_Event_request
{
public:
  explicit Init_FollowPath_GetResult_Event_request(::action_interfaces::action::FollowPath_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_FollowPath_GetResult_Event_response request(::action_interfaces::action::FollowPath_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_FollowPath_GetResult_Event_response(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Event msg_;
};

class Init_FollowPath_GetResult_Event_info
{
public:
  Init_FollowPath_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_GetResult_Event_request info(::action_interfaces::action::FollowPath_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_FollowPath_GetResult_Event_request(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_GetResult_Event>()
{
  return action_interfaces::action::builder::Init_FollowPath_GetResult_Event_info();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_FollowPath_FeedbackMessage_feedback
{
public:
  explicit Init_FollowPath_FeedbackMessage_feedback(::action_interfaces::action::FollowPath_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::FollowPath_FeedbackMessage feedback(::action_interfaces::action::FollowPath_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_FeedbackMessage msg_;
};

class Init_FollowPath_FeedbackMessage_goal_id
{
public:
  Init_FollowPath_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_FollowPath_FeedbackMessage_feedback goal_id(::action_interfaces::action::FollowPath_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_FollowPath_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_interfaces::action::FollowPath_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::FollowPath_FeedbackMessage>()
{
  return action_interfaces::action::builder::Init_FollowPath_FeedbackMessage_goal_id();
}

}  // namespace action_interfaces

#endif  // ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__BUILDER_HPP_
