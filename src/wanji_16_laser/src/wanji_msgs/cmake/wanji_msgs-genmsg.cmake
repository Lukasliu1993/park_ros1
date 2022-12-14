# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "wanji_msgs: 2 messages, 0 services")

set(MSG_I_FLAGS "-Iwanji_msgs:/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg;-Istd_msgs:/opt/ros/indigo/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(genlisp REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(wanji_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg" NAME_WE)
add_custom_target(_wanji_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wanji_msgs" "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg" "std_msgs/Header:wanji_msgs/WanjiPacket"
)

get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg" NAME_WE)
add_custom_target(_wanji_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "wanji_msgs" "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg" ""
)

#
#  langs = gencpp;genlisp;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wanji_msgs
)
_generate_msg_cpp(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wanji_msgs
)

### Generating Services

### Generating Module File
_generate_module_cpp(wanji_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wanji_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(wanji_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(wanji_msgs_generate_messages wanji_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_cpp _wanji_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_cpp _wanji_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wanji_msgs_gencpp)
add_dependencies(wanji_msgs_gencpp wanji_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wanji_msgs_generate_messages_cpp)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wanji_msgs
)
_generate_msg_lisp(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wanji_msgs
)

### Generating Services

### Generating Module File
_generate_module_lisp(wanji_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wanji_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(wanji_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(wanji_msgs_generate_messages wanji_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_lisp _wanji_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_lisp _wanji_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wanji_msgs_genlisp)
add_dependencies(wanji_msgs_genlisp wanji_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wanji_msgs_generate_messages_lisp)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/indigo/share/std_msgs/cmake/../msg/Header.msg;/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs
)
_generate_msg_py(wanji_msgs
  "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs
)

### Generating Services

### Generating Module File
_generate_module_py(wanji_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(wanji_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(wanji_msgs_generate_messages wanji_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiScan.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_py _wanji_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/exbot/catkin_ws/src/wanji-master/wanji_msgs/msg/WanjiPacket.msg" NAME_WE)
add_dependencies(wanji_msgs_generate_messages_py _wanji_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(wanji_msgs_genpy)
add_dependencies(wanji_msgs_genpy wanji_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS wanji_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wanji_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/wanji_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(wanji_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wanji_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/wanji_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(wanji_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/wanji_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(wanji_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
