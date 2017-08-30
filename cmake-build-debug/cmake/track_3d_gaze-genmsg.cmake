# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "track_3d_gaze: 1 messages, 0 services")

set(MSG_I_FLAGS "-Itrack_3d_gaze:/home/alexandros/catkin_ws/src/track_3d_gaze/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(track_3d_gaze_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg" NAME_WE)
add_custom_target(_track_3d_gaze_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "track_3d_gaze" "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(track_3d_gaze
  "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/track_3d_gaze
)

### Generating Services

### Generating Module File
_generate_module_cpp(track_3d_gaze
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/track_3d_gaze
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(track_3d_gaze_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(track_3d_gaze_generate_messages track_3d_gaze_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg" NAME_WE)
add_dependencies(track_3d_gaze_generate_messages_cpp _track_3d_gaze_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(track_3d_gaze_gencpp)
add_dependencies(track_3d_gaze_gencpp track_3d_gaze_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS track_3d_gaze_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(track_3d_gaze
  "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/track_3d_gaze
)

### Generating Services

### Generating Module File
_generate_module_lisp(track_3d_gaze
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/track_3d_gaze
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(track_3d_gaze_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(track_3d_gaze_generate_messages track_3d_gaze_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg" NAME_WE)
add_dependencies(track_3d_gaze_generate_messages_lisp _track_3d_gaze_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(track_3d_gaze_genlisp)
add_dependencies(track_3d_gaze_genlisp track_3d_gaze_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS track_3d_gaze_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(track_3d_gaze
  "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/track_3d_gaze
)

### Generating Services

### Generating Module File
_generate_module_py(track_3d_gaze
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/track_3d_gaze
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(track_3d_gaze_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(track_3d_gaze_generate_messages track_3d_gaze_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/alexandros/catkin_ws/src/track_3d_gaze/msg/Message1.msg" NAME_WE)
add_dependencies(track_3d_gaze_generate_messages_py _track_3d_gaze_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(track_3d_gaze_genpy)
add_dependencies(track_3d_gaze_genpy track_3d_gaze_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS track_3d_gaze_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/track_3d_gaze)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/track_3d_gaze
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(track_3d_gaze_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/track_3d_gaze)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/track_3d_gaze
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(track_3d_gaze_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/track_3d_gaze)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/track_3d_gaze\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/track_3d_gaze
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(track_3d_gaze_generate_messages_py std_msgs_generate_messages_py)
endif()
