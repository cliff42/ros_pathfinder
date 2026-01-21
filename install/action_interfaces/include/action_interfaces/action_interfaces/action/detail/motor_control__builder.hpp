// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from action_interfaces:action/MotorControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "action_interfaces/action/motor_control.hpp"


#ifndef ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__BUILDER_HPP_
#define ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "action_interfaces/action/detail/motor_control__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_Goal_plan
{
public:
  Init_MotorControl_Goal_plan()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::MotorControl_Goal plan(::action_interfaces::action::MotorControl_Goal::_plan_type arg)
  {
    msg_.plan = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_Goal msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_Goal>()
{
  return action_interfaces::action::builder::Init_MotorControl_Goal_plan();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_Result_success
{
public:
  Init_MotorControl_Result_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::MotorControl_Result success(::action_interfaces::action::MotorControl_Result::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_Result msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_Result>()
{
  return action_interfaces::action::builder::Init_MotorControl_Result_success();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_Feedback_distance_remaining
{
public:
  Init_MotorControl_Feedback_distance_remaining()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::MotorControl_Feedback distance_remaining(::action_interfaces::action::MotorControl_Feedback::_distance_remaining_type arg)
  {
    msg_.distance_remaining = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_Feedback msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_Feedback>()
{
  return action_interfaces::action::builder::Init_MotorControl_Feedback_distance_remaining();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_SendGoal_Request_goal
{
public:
  explicit Init_MotorControl_SendGoal_Request_goal(::action_interfaces::action::MotorControl_SendGoal_Request & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_SendGoal_Request goal(::action_interfaces::action::MotorControl_SendGoal_Request::_goal_type arg)
  {
    msg_.goal = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Request msg_;
};

class Init_MotorControl_SendGoal_Request_goal_id
{
public:
  Init_MotorControl_SendGoal_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_SendGoal_Request_goal goal_id(::action_interfaces::action::MotorControl_SendGoal_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MotorControl_SendGoal_Request_goal(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_SendGoal_Request>()
{
  return action_interfaces::action::builder::Init_MotorControl_SendGoal_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_SendGoal_Response_stamp
{
public:
  explicit Init_MotorControl_SendGoal_Response_stamp(::action_interfaces::action::MotorControl_SendGoal_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_SendGoal_Response stamp(::action_interfaces::action::MotorControl_SendGoal_Response::_stamp_type arg)
  {
    msg_.stamp = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Response msg_;
};

class Init_MotorControl_SendGoal_Response_accepted
{
public:
  Init_MotorControl_SendGoal_Response_accepted()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_SendGoal_Response_stamp accepted(::action_interfaces::action::MotorControl_SendGoal_Response::_accepted_type arg)
  {
    msg_.accepted = std::move(arg);
    return Init_MotorControl_SendGoal_Response_stamp(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_SendGoal_Response>()
{
  return action_interfaces::action::builder::Init_MotorControl_SendGoal_Response_accepted();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_SendGoal_Event_response
{
public:
  explicit Init_MotorControl_SendGoal_Event_response(::action_interfaces::action::MotorControl_SendGoal_Event & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_SendGoal_Event response(::action_interfaces::action::MotorControl_SendGoal_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Event msg_;
};

class Init_MotorControl_SendGoal_Event_request
{
public:
  explicit Init_MotorControl_SendGoal_Event_request(::action_interfaces::action::MotorControl_SendGoal_Event & msg)
  : msg_(msg)
  {}
  Init_MotorControl_SendGoal_Event_response request(::action_interfaces::action::MotorControl_SendGoal_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MotorControl_SendGoal_Event_response(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Event msg_;
};

class Init_MotorControl_SendGoal_Event_info
{
public:
  Init_MotorControl_SendGoal_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_SendGoal_Event_request info(::action_interfaces::action::MotorControl_SendGoal_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MotorControl_SendGoal_Event_request(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_SendGoal_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_SendGoal_Event>()
{
  return action_interfaces::action::builder::Init_MotorControl_SendGoal_Event_info();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_GetResult_Request_goal_id
{
public:
  Init_MotorControl_GetResult_Request_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::action_interfaces::action::MotorControl_GetResult_Request goal_id(::action_interfaces::action::MotorControl_GetResult_Request::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Request msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_GetResult_Request>()
{
  return action_interfaces::action::builder::Init_MotorControl_GetResult_Request_goal_id();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_GetResult_Response_result
{
public:
  explicit Init_MotorControl_GetResult_Response_result(::action_interfaces::action::MotorControl_GetResult_Response & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_GetResult_Response result(::action_interfaces::action::MotorControl_GetResult_Response::_result_type arg)
  {
    msg_.result = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Response msg_;
};

class Init_MotorControl_GetResult_Response_status
{
public:
  Init_MotorControl_GetResult_Response_status()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_GetResult_Response_result status(::action_interfaces::action::MotorControl_GetResult_Response::_status_type arg)
  {
    msg_.status = std::move(arg);
    return Init_MotorControl_GetResult_Response_result(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Response msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_GetResult_Response>()
{
  return action_interfaces::action::builder::Init_MotorControl_GetResult_Response_status();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_GetResult_Event_response
{
public:
  explicit Init_MotorControl_GetResult_Event_response(::action_interfaces::action::MotorControl_GetResult_Event & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_GetResult_Event response(::action_interfaces::action::MotorControl_GetResult_Event::_response_type arg)
  {
    msg_.response = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Event msg_;
};

class Init_MotorControl_GetResult_Event_request
{
public:
  explicit Init_MotorControl_GetResult_Event_request(::action_interfaces::action::MotorControl_GetResult_Event & msg)
  : msg_(msg)
  {}
  Init_MotorControl_GetResult_Event_response request(::action_interfaces::action::MotorControl_GetResult_Event::_request_type arg)
  {
    msg_.request = std::move(arg);
    return Init_MotorControl_GetResult_Event_response(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Event msg_;
};

class Init_MotorControl_GetResult_Event_info
{
public:
  Init_MotorControl_GetResult_Event_info()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_GetResult_Event_request info(::action_interfaces::action::MotorControl_GetResult_Event::_info_type arg)
  {
    msg_.info = std::move(arg);
    return Init_MotorControl_GetResult_Event_request(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_GetResult_Event msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_GetResult_Event>()
{
  return action_interfaces::action::builder::Init_MotorControl_GetResult_Event_info();
}

}  // namespace action_interfaces


namespace action_interfaces
{

namespace action
{

namespace builder
{

class Init_MotorControl_FeedbackMessage_feedback
{
public:
  explicit Init_MotorControl_FeedbackMessage_feedback(::action_interfaces::action::MotorControl_FeedbackMessage & msg)
  : msg_(msg)
  {}
  ::action_interfaces::action::MotorControl_FeedbackMessage feedback(::action_interfaces::action::MotorControl_FeedbackMessage::_feedback_type arg)
  {
    msg_.feedback = std::move(arg);
    return std::move(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_FeedbackMessage msg_;
};

class Init_MotorControl_FeedbackMessage_goal_id
{
public:
  Init_MotorControl_FeedbackMessage_goal_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MotorControl_FeedbackMessage_feedback goal_id(::action_interfaces::action::MotorControl_FeedbackMessage::_goal_id_type arg)
  {
    msg_.goal_id = std::move(arg);
    return Init_MotorControl_FeedbackMessage_feedback(msg_);
  }

private:
  ::action_interfaces::action::MotorControl_FeedbackMessage msg_;
};

}  // namespace builder

}  // namespace action

template<typename MessageType>
auto build();

template<>
inline
auto build<::action_interfaces::action::MotorControl_FeedbackMessage>()
{
  return action_interfaces::action::builder::Init_MotorControl_FeedbackMessage_goal_id();
}

}  // namespace action_interfaces

#endif  // ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__BUILDER_HPP_
