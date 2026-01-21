// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_interfaces:action/MotorControl.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "action_interfaces/action/motor_control.h"


#ifndef ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__STRUCT_H_
#define ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'plan'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_Goal
{
  rosidl_runtime_c__double__Sequence plan;
} action_interfaces__action__MotorControl_Goal;

// Struct for a sequence of action_interfaces__action__MotorControl_Goal.
typedef struct action_interfaces__action__MotorControl_Goal__Sequence
{
  action_interfaces__action__MotorControl_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_Goal__Sequence;

// Constants defined in the message

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_Result
{
  bool success;
} action_interfaces__action__MotorControl_Result;

// Struct for a sequence of action_interfaces__action__MotorControl_Result.
typedef struct action_interfaces__action__MotorControl_Result__Sequence
{
  action_interfaces__action__MotorControl_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_Feedback
{
  double distance_remaining;
} action_interfaces__action__MotorControl_Feedback;

// Struct for a sequence of action_interfaces__action__MotorControl_Feedback.
typedef struct action_interfaces__action__MotorControl_Feedback__Sequence
{
  action_interfaces__action__MotorControl_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_interfaces/action/detail/motor_control__struct.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__MotorControl_Goal goal;
} action_interfaces__action__MotorControl_SendGoal_Request;

// Struct for a sequence of action_interfaces__action__MotorControl_SendGoal_Request.
typedef struct action_interfaces__action__MotorControl_SendGoal_Request__Sequence
{
  action_interfaces__action__MotorControl_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_interfaces__action__MotorControl_SendGoal_Response;

// Struct for a sequence of action_interfaces__action__MotorControl_SendGoal_Response.
typedef struct action_interfaces__action__MotorControl_SendGoal_Response__Sequence
{
  action_interfaces__action__MotorControl_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  action_interfaces__action__MotorControl_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  action_interfaces__action__MotorControl_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  action_interfaces__action__MotorControl_SendGoal_Request__Sequence request;
  action_interfaces__action__MotorControl_SendGoal_Response__Sequence response;
} action_interfaces__action__MotorControl_SendGoal_Event;

// Struct for a sequence of action_interfaces__action__MotorControl_SendGoal_Event.
typedef struct action_interfaces__action__MotorControl_SendGoal_Event__Sequence
{
  action_interfaces__action__MotorControl_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_interfaces__action__MotorControl_GetResult_Request;

// Struct for a sequence of action_interfaces__action__MotorControl_GetResult_Request.
typedef struct action_interfaces__action__MotorControl_GetResult_Request__Sequence
{
  action_interfaces__action__MotorControl_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_interfaces/action/detail/motor_control__struct.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_GetResult_Response
{
  int8_t status;
  action_interfaces__action__MotorControl_Result result;
} action_interfaces__action__MotorControl_GetResult_Response;

// Struct for a sequence of action_interfaces__action__MotorControl_GetResult_Response.
typedef struct action_interfaces__action__MotorControl_GetResult_Response__Sequence
{
  action_interfaces__action__MotorControl_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  action_interfaces__action__MotorControl_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  action_interfaces__action__MotorControl_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  action_interfaces__action__MotorControl_GetResult_Request__Sequence request;
  action_interfaces__action__MotorControl_GetResult_Response__Sequence response;
} action_interfaces__action__MotorControl_GetResult_Event;

// Struct for a sequence of action_interfaces__action__MotorControl_GetResult_Event.
typedef struct action_interfaces__action__MotorControl_GetResult_Event__Sequence
{
  action_interfaces__action__MotorControl_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_interfaces/action/detail/motor_control__struct.h"

/// Struct defined in action/MotorControl in the package action_interfaces.
typedef struct action_interfaces__action__MotorControl_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__MotorControl_Feedback feedback;
} action_interfaces__action__MotorControl_FeedbackMessage;

// Struct for a sequence of action_interfaces__action__MotorControl_FeedbackMessage.
typedef struct action_interfaces__action__MotorControl_FeedbackMessage__Sequence
{
  action_interfaces__action__MotorControl_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__MotorControl_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_INTERFACES__ACTION__DETAIL__MOTOR_CONTROL__STRUCT_H_
