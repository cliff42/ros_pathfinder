// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_interfaces:action/FollowPath.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "action_interfaces/action/follow_path.h"


#ifndef ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
#define ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'path'
#include "nav_msgs/msg/detail/path__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_Goal
{
  nav_msgs__msg__Path path;
} action_interfaces__action__FollowPath_Goal;

// Struct for a sequence of action_interfaces__action__FollowPath_Goal.
typedef struct action_interfaces__action__FollowPath_Goal__Sequence
{
  action_interfaces__action__FollowPath_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_Goal__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'message'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_Result
{
  bool success;
  rosidl_runtime_c__String message;
} action_interfaces__action__FollowPath_Result;

// Struct for a sequence of action_interfaces__action__FollowPath_Result.
typedef struct action_interfaces__action__FollowPath_Result__Sequence
{
  action_interfaces__action__FollowPath_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_Result__Sequence;

// Constants defined in the message

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_Feedback
{
  int32_t current_waypoint;
  int32_t total_waypoints;
  double distance_to_goal;
} action_interfaces__action__FollowPath_Feedback;

// Struct for a sequence of action_interfaces__action__FollowPath_Feedback.
typedef struct action_interfaces__action__FollowPath_Feedback__Sequence
{
  action_interfaces__action__FollowPath_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_Feedback__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_interfaces/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__FollowPath_Goal goal;
} action_interfaces__action__FollowPath_SendGoal_Request;

// Struct for a sequence of action_interfaces__action__FollowPath_SendGoal_Request.
typedef struct action_interfaces__action__FollowPath_SendGoal_Request__Sequence
{
  action_interfaces__action__FollowPath_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_SendGoal_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_interfaces__action__FollowPath_SendGoal_Response;

// Struct for a sequence of action_interfaces__action__FollowPath_SendGoal_Response.
typedef struct action_interfaces__action__FollowPath_SendGoal_Response__Sequence
{
  action_interfaces__action__FollowPath_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_SendGoal_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
#include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  action_interfaces__action__FollowPath_SendGoal_Event__request__MAX_SIZE = 1
};
// response
enum
{
  action_interfaces__action__FollowPath_SendGoal_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_SendGoal_Event
{
  service_msgs__msg__ServiceEventInfo info;
  action_interfaces__action__FollowPath_SendGoal_Request__Sequence request;
  action_interfaces__action__FollowPath_SendGoal_Response__Sequence response;
} action_interfaces__action__FollowPath_SendGoal_Event;

// Struct for a sequence of action_interfaces__action__FollowPath_SendGoal_Event.
typedef struct action_interfaces__action__FollowPath_SendGoal_Event__Sequence
{
  action_interfaces__action__FollowPath_SendGoal_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_SendGoal_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_interfaces__action__FollowPath_GetResult_Request;

// Struct for a sequence of action_interfaces__action__FollowPath_GetResult_Request.
typedef struct action_interfaces__action__FollowPath_GetResult_Request__Sequence
{
  action_interfaces__action__FollowPath_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_GetResult_Request__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_interfaces/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_GetResult_Response
{
  int8_t status;
  action_interfaces__action__FollowPath_Result result;
} action_interfaces__action__FollowPath_GetResult_Response;

// Struct for a sequence of action_interfaces__action__FollowPath_GetResult_Response.
typedef struct action_interfaces__action__FollowPath_GetResult_Response__Sequence
{
  action_interfaces__action__FollowPath_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_GetResult_Response__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'info'
// already included above
// #include "service_msgs/msg/detail/service_event_info__struct.h"

// constants for array fields with an upper bound
// request
enum
{
  action_interfaces__action__FollowPath_GetResult_Event__request__MAX_SIZE = 1
};
// response
enum
{
  action_interfaces__action__FollowPath_GetResult_Event__response__MAX_SIZE = 1
};

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_GetResult_Event
{
  service_msgs__msg__ServiceEventInfo info;
  action_interfaces__action__FollowPath_GetResult_Request__Sequence request;
  action_interfaces__action__FollowPath_GetResult_Response__Sequence response;
} action_interfaces__action__FollowPath_GetResult_Event;

// Struct for a sequence of action_interfaces__action__FollowPath_GetResult_Event.
typedef struct action_interfaces__action__FollowPath_GetResult_Event__Sequence
{
  action_interfaces__action__FollowPath_GetResult_Event * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_GetResult_Event__Sequence;

// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_interfaces/action/detail/follow_path__struct.h"

/// Struct defined in action/FollowPath in the package action_interfaces.
typedef struct action_interfaces__action__FollowPath_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_interfaces__action__FollowPath_Feedback feedback;
} action_interfaces__action__FollowPath_FeedbackMessage;

// Struct for a sequence of action_interfaces__action__FollowPath_FeedbackMessage.
typedef struct action_interfaces__action__FollowPath_FeedbackMessage__Sequence
{
  action_interfaces__action__FollowPath_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_interfaces__action__FollowPath_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_INTERFACES__ACTION__DETAIL__FOLLOW_PATH__STRUCT_H_
