// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from control_msgs:action/ParallelGripperCommand.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_Goal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_Goal_type_support_ids_t;

static const _ParallelGripperCommand_Goal_type_support_ids_t _ParallelGripperCommand_Goal_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_Goal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_Goal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_Goal_type_support_symbol_names_t _ParallelGripperCommand_Goal_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_Goal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_Goal)),
  }
};

typedef struct _ParallelGripperCommand_Goal_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_Goal_type_support_data_t;

static _ParallelGripperCommand_Goal_type_support_data_t _ParallelGripperCommand_Goal_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_Goal_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_Goal_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_Goal_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_Goal_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_Goal_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_Goal_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Goal>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_Goal_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_Goal)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Goal>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_Result_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_Result_type_support_ids_t;

static const _ParallelGripperCommand_Result_type_support_ids_t _ParallelGripperCommand_Result_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_Result_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_Result_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_Result_type_support_symbol_names_t _ParallelGripperCommand_Result_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_Result)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_Result)),
  }
};

typedef struct _ParallelGripperCommand_Result_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_Result_type_support_data_t;

static _ParallelGripperCommand_Result_type_support_data_t _ParallelGripperCommand_Result_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_Result_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_Result_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_Result_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_Result_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_Result_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_Result_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Result>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_Result_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_Result)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Result>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_Feedback_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_Feedback_type_support_ids_t;

static const _ParallelGripperCommand_Feedback_type_support_ids_t _ParallelGripperCommand_Feedback_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_Feedback_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_Feedback_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_Feedback_type_support_symbol_names_t _ParallelGripperCommand_Feedback_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_Feedback)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_Feedback)),
  }
};

typedef struct _ParallelGripperCommand_Feedback_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_Feedback_type_support_data_t;

static _ParallelGripperCommand_Feedback_type_support_data_t _ParallelGripperCommand_Feedback_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_Feedback_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_Feedback_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_Feedback_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_Feedback_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_Feedback_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_Feedback_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Feedback>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_Feedback_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_Feedback)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_Feedback>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_SendGoal_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_SendGoal_Request_type_support_ids_t;

static const _ParallelGripperCommand_SendGoal_Request_type_support_ids_t _ParallelGripperCommand_SendGoal_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_SendGoal_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_SendGoal_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_SendGoal_Request_type_support_symbol_names_t _ParallelGripperCommand_SendGoal_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Request)),
  }
};

typedef struct _ParallelGripperCommand_SendGoal_Request_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_SendGoal_Request_type_support_data_t;

static _ParallelGripperCommand_SendGoal_Request_type_support_data_t _ParallelGripperCommand_SendGoal_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_SendGoal_Request_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_SendGoal_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_SendGoal_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_SendGoal_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_SendGoal_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_SendGoal_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_SendGoal_Request>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_SendGoal_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Request)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_SendGoal_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_SendGoal_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_SendGoal_Response_type_support_ids_t;

static const _ParallelGripperCommand_SendGoal_Response_type_support_ids_t _ParallelGripperCommand_SendGoal_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_SendGoal_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_SendGoal_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_SendGoal_Response_type_support_symbol_names_t _ParallelGripperCommand_SendGoal_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Response)),
  }
};

typedef struct _ParallelGripperCommand_SendGoal_Response_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_SendGoal_Response_type_support_data_t;

static _ParallelGripperCommand_SendGoal_Response_type_support_data_t _ParallelGripperCommand_SendGoal_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_SendGoal_Response_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_SendGoal_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_SendGoal_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_SendGoal_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_SendGoal_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_SendGoal_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_SendGoal_Response>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_SendGoal_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_SendGoal_Response)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_SendGoal_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
#include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_SendGoal_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_SendGoal_type_support_ids_t;

static const _ParallelGripperCommand_SendGoal_type_support_ids_t _ParallelGripperCommand_SendGoal_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_SendGoal_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_SendGoal_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_SendGoal_type_support_symbol_names_t _ParallelGripperCommand_SendGoal_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_SendGoal)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_SendGoal)),
  }
};

typedef struct _ParallelGripperCommand_SendGoal_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_SendGoal_type_support_data_t;

static _ParallelGripperCommand_SendGoal_type_support_data_t _ParallelGripperCommand_SendGoal_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_SendGoal_service_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_SendGoal_service_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_SendGoal_service_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_SendGoal_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ParallelGripperCommand_SendGoal_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_SendGoal_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<control_msgs::action::ParallelGripperCommand_SendGoal>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_SendGoal_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_GetResult_Request_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_GetResult_Request_type_support_ids_t;

static const _ParallelGripperCommand_GetResult_Request_type_support_ids_t _ParallelGripperCommand_GetResult_Request_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_GetResult_Request_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_GetResult_Request_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_GetResult_Request_type_support_symbol_names_t _ParallelGripperCommand_GetResult_Request_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Request)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Request)),
  }
};

typedef struct _ParallelGripperCommand_GetResult_Request_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_GetResult_Request_type_support_data_t;

static _ParallelGripperCommand_GetResult_Request_type_support_data_t _ParallelGripperCommand_GetResult_Request_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_GetResult_Request_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_GetResult_Request_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_GetResult_Request_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_GetResult_Request_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_GetResult_Request_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_GetResult_Request_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_GetResult_Request>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_GetResult_Request_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Request)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_GetResult_Request>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_GetResult_Response_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_GetResult_Response_type_support_ids_t;

static const _ParallelGripperCommand_GetResult_Response_type_support_ids_t _ParallelGripperCommand_GetResult_Response_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_GetResult_Response_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_GetResult_Response_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_GetResult_Response_type_support_symbol_names_t _ParallelGripperCommand_GetResult_Response_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Response)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Response)),
  }
};

typedef struct _ParallelGripperCommand_GetResult_Response_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_GetResult_Response_type_support_data_t;

static _ParallelGripperCommand_GetResult_Response_type_support_data_t _ParallelGripperCommand_GetResult_Response_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_GetResult_Response_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_GetResult_Response_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_GetResult_Response_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_GetResult_Response_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_GetResult_Response_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_GetResult_Response_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_GetResult_Response>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_GetResult_Response_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_GetResult_Response)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_GetResult_Response>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/service_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_GetResult_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_GetResult_type_support_ids_t;

static const _ParallelGripperCommand_GetResult_type_support_ids_t _ParallelGripperCommand_GetResult_service_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_GetResult_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_GetResult_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_GetResult_type_support_symbol_names_t _ParallelGripperCommand_GetResult_service_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_GetResult)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__SERVICE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_GetResult)),
  }
};

typedef struct _ParallelGripperCommand_GetResult_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_GetResult_type_support_data_t;

static _ParallelGripperCommand_GetResult_type_support_data_t _ParallelGripperCommand_GetResult_service_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_GetResult_service_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_GetResult_service_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_GetResult_service_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_GetResult_service_typesupport_data.data[0],
};

static const rosidl_service_type_support_t ParallelGripperCommand_GetResult_service_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_GetResult_service_typesupport_map),
  ::rosidl_typesupport_cpp::get_service_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_service_type_support_t *
get_service_type_support_handle<control_msgs::action::ParallelGripperCommand_GetResult>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_GetResult_service_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp

// already included above
// #include "cstddef"
// already included above
// #include "rosidl_runtime_c/message_type_support_struct.h"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/identifier.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_c/type_support_map.h"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
// already included above
// #include "rosidl_typesupport_interface/macros.h"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

typedef struct _ParallelGripperCommand_FeedbackMessage_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _ParallelGripperCommand_FeedbackMessage_type_support_ids_t;

static const _ParallelGripperCommand_FeedbackMessage_type_support_ids_t _ParallelGripperCommand_FeedbackMessage_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _ParallelGripperCommand_FeedbackMessage_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _ParallelGripperCommand_FeedbackMessage_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _ParallelGripperCommand_FeedbackMessage_type_support_symbol_names_t _ParallelGripperCommand_FeedbackMessage_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, control_msgs, action, ParallelGripperCommand_FeedbackMessage)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, control_msgs, action, ParallelGripperCommand_FeedbackMessage)),
  }
};

typedef struct _ParallelGripperCommand_FeedbackMessage_type_support_data_t
{
  void * data[2];
} _ParallelGripperCommand_FeedbackMessage_type_support_data_t;

static _ParallelGripperCommand_FeedbackMessage_type_support_data_t _ParallelGripperCommand_FeedbackMessage_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _ParallelGripperCommand_FeedbackMessage_message_typesupport_map = {
  2,
  "control_msgs",
  &_ParallelGripperCommand_FeedbackMessage_message_typesupport_ids.typesupport_identifier[0],
  &_ParallelGripperCommand_FeedbackMessage_message_typesupport_symbol_names.symbol_name[0],
  &_ParallelGripperCommand_FeedbackMessage_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t ParallelGripperCommand_FeedbackMessage_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_ParallelGripperCommand_FeedbackMessage_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_FeedbackMessage>()
{
  return &::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_FeedbackMessage_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, control_msgs, action, ParallelGripperCommand_FeedbackMessage)() {
  return get_message_type_support_handle<control_msgs::action::ParallelGripperCommand_FeedbackMessage>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp

#include "action_msgs/msg/goal_status_array.hpp"
#include "action_msgs/srv/cancel_goal.hpp"
// already included above
// #include "control_msgs/action/detail/parallel_gripper_command__struct.hpp"
// already included above
// #include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_typesupport_cpp/action_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/message_type_support.hpp"
// already included above
// #include "rosidl_typesupport_cpp/service_type_support.hpp"

namespace control_msgs
{

namespace action
{

namespace rosidl_typesupport_cpp
{

static rosidl_action_type_support_t ParallelGripperCommand_action_type_support_handle = {
  NULL, NULL, NULL, NULL, NULL};

}  // namespace rosidl_typesupport_cpp

}  // namespace action

}  // namespace control_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_action_type_support_t *
get_action_type_support_handle<control_msgs::action::ParallelGripperCommand>()
{
  using ::control_msgs::action::rosidl_typesupport_cpp::ParallelGripperCommand_action_type_support_handle;
  // Thread-safe by always writing the same values to the static struct
  ParallelGripperCommand_action_type_support_handle.goal_service_type_support = get_service_type_support_handle<::control_msgs::action::ParallelGripperCommand::Impl::SendGoalService>();
  ParallelGripperCommand_action_type_support_handle.result_service_type_support = get_service_type_support_handle<::control_msgs::action::ParallelGripperCommand::Impl::GetResultService>();
  ParallelGripperCommand_action_type_support_handle.cancel_service_type_support = get_service_type_support_handle<::control_msgs::action::ParallelGripperCommand::Impl::CancelGoalService>();
  ParallelGripperCommand_action_type_support_handle.feedback_message_type_support = get_message_type_support_handle<::control_msgs::action::ParallelGripperCommand::Impl::FeedbackMessage>();
  ParallelGripperCommand_action_type_support_handle.status_message_type_support = get_message_type_support_handle<::control_msgs::action::ParallelGripperCommand::Impl::GoalStatusMessage>();
  return &ParallelGripperCommand_action_type_support_handle;
}

}  // namespace rosidl_typesupport_cpp
