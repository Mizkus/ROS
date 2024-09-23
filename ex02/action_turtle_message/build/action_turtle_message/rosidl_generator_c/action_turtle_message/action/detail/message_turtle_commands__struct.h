// NOLINT: This file starts with a BOM since it contain non-ASCII characters
// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from action_turtle_message:action/MessageTurtleCommands.idl
// generated code does not contain a copyright notice

#ifndef ACTION_TURTLE_MESSAGE__ACTION__DETAIL__MESSAGE_TURTLE_COMMANDS__STRUCT_H_
#define ACTION_TURTLE_MESSAGE__ACTION__DETAIL__MESSAGE_TURTLE_COMMANDS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_Goal
{
  /// “forward”,”turn_left”, “turn_right”
  rosidl_runtime_c__String command;
  /// расстояние в метрах которое должна проехать черепаха
  int32_t s;
  /// угол в градусах на который должна повернуть черепаха
  int32_t angle;
} action_turtle_message__action__MessageTurtleCommands_Goal;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_Goal.
typedef struct action_turtle_message__action__MessageTurtleCommands_Goal__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_Goal * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_Goal__Sequence;


// Constants defined in the message

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_Result
{
  /// true - черепаха выполнила команду, false - черепаха не смогла выполнить команду, например потому что выполнение команды прервали
  bool result;
} action_turtle_message__action__MessageTurtleCommands_Result;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_Result.
typedef struct action_turtle_message__action__MessageTurtleCommands_Result__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_Result * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_Result__Sequence;


// Constants defined in the message

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_Feedback
{
  /// пройденное черепахой расстояние в метрах с момента начала выполнения команды
  int32_t odom;
} action_turtle_message__action__MessageTurtleCommands_Feedback;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_Feedback.
typedef struct action_turtle_message__action__MessageTurtleCommands_Feedback__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_Feedback * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_Feedback__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
#include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'goal'
#include "action_turtle_message/action/detail/message_turtle_commands__struct.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_SendGoal_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_turtle_message__action__MessageTurtleCommands_Goal goal;
} action_turtle_message__action__MessageTurtleCommands_SendGoal_Request;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_SendGoal_Request.
typedef struct action_turtle_message__action__MessageTurtleCommands_SendGoal_Request__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_SendGoal_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_SendGoal_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'stamp'
#include "builtin_interfaces/msg/detail/time__struct.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_SendGoal_Response
{
  bool accepted;
  builtin_interfaces__msg__Time stamp;
} action_turtle_message__action__MessageTurtleCommands_SendGoal_Response;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_SendGoal_Response.
typedef struct action_turtle_message__action__MessageTurtleCommands_SendGoal_Response__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_SendGoal_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_SendGoal_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_GetResult_Request
{
  unique_identifier_msgs__msg__UUID goal_id;
} action_turtle_message__action__MessageTurtleCommands_GetResult_Request;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_GetResult_Request.
typedef struct action_turtle_message__action__MessageTurtleCommands_GetResult_Request__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_GetResult_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_GetResult_Request__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'result'
// already included above
// #include "action_turtle_message/action/detail/message_turtle_commands__struct.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_GetResult_Response
{
  int8_t status;
  action_turtle_message__action__MessageTurtleCommands_Result result;
} action_turtle_message__action__MessageTurtleCommands_GetResult_Response;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_GetResult_Response.
typedef struct action_turtle_message__action__MessageTurtleCommands_GetResult_Response__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_GetResult_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_GetResult_Response__Sequence;


// Constants defined in the message

// Include directives for member types
// Member 'goal_id'
// already included above
// #include "unique_identifier_msgs/msg/detail/uuid__struct.h"
// Member 'feedback'
// already included above
// #include "action_turtle_message/action/detail/message_turtle_commands__struct.h"

/// Struct defined in action/MessageTurtleCommands in the package action_turtle_message.
typedef struct action_turtle_message__action__MessageTurtleCommands_FeedbackMessage
{
  unique_identifier_msgs__msg__UUID goal_id;
  action_turtle_message__action__MessageTurtleCommands_Feedback feedback;
} action_turtle_message__action__MessageTurtleCommands_FeedbackMessage;

// Struct for a sequence of action_turtle_message__action__MessageTurtleCommands_FeedbackMessage.
typedef struct action_turtle_message__action__MessageTurtleCommands_FeedbackMessage__Sequence
{
  action_turtle_message__action__MessageTurtleCommands_FeedbackMessage * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} action_turtle_message__action__MessageTurtleCommands_FeedbackMessage__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ACTION_TURTLE_MESSAGE__ACTION__DETAIL__MESSAGE_TURTLE_COMMANDS__STRUCT_H_
