# generated from genmsg/cmake/pkg-genmsg.cmake.em

message(STATUS "robot_msgs: 10 messages, 1 services")

set(MSG_I_FLAGS "-Irobot_msgs:/home/hj/sentry_ros_25/src/robot_msgs/msg;-Istd_msgs:/opt/ros/noetic/share/std_msgs/cmake/../msg")

# Find all generators
find_package(gencpp REQUIRED)
find_package(geneus REQUIRED)
find_package(genlisp REQUIRED)
find_package(gennodejs REQUIRED)
find_package(genpy REQUIRED)

add_custom_target(robot_msgs_generate_messages ALL)

# verify that message/service dependencies have not changed since configure



get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" "std_msgs/Header"
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" ""
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" ""
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" ""
)

get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_custom_target(_robot_msgs_generate_messages_check_deps_${_filename}
  COMMAND ${CATKIN_ENV} ${PYTHON_EXECUTABLE} ${GENMSG_CHECK_DEPS_SCRIPT} "robot_msgs" "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" ""
)

#
#  langs = gencpp;geneus;genlisp;gennodejs;genpy
#

### Section generating for lang: gencpp
### Generating Messages
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)
_generate_msg_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)

### Generating Services
_generate_srv_cpp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
)

### Generating Module File
_generate_module_cpp(robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
  "${ALL_GEN_OUTPUT_FILES_cpp}"
)

add_custom_target(robot_msgs_generate_messages_cpp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_cpp}
)
add_dependencies(robot_msgs_generate_messages robot_msgs_generate_messages_cpp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_dependencies(robot_msgs_generate_messages_cpp _robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_msgs_gencpp)
add_dependencies(robot_msgs_gencpp robot_msgs_generate_messages_cpp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_msgs_generate_messages_cpp)

### Section generating for lang: geneus
### Generating Messages
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)
_generate_msg_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)

### Generating Services
_generate_srv_eus(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
)

### Generating Module File
_generate_module_eus(robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
  "${ALL_GEN_OUTPUT_FILES_eus}"
)

add_custom_target(robot_msgs_generate_messages_eus
  DEPENDS ${ALL_GEN_OUTPUT_FILES_eus}
)
add_dependencies(robot_msgs_generate_messages robot_msgs_generate_messages_eus)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_dependencies(robot_msgs_generate_messages_eus _robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_msgs_geneus)
add_dependencies(robot_msgs_geneus robot_msgs_generate_messages_eus)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_msgs_generate_messages_eus)

### Section generating for lang: genlisp
### Generating Messages
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)
_generate_msg_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)

### Generating Services
_generate_srv_lisp(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
)

### Generating Module File
_generate_module_lisp(robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
  "${ALL_GEN_OUTPUT_FILES_lisp}"
)

add_custom_target(robot_msgs_generate_messages_lisp
  DEPENDS ${ALL_GEN_OUTPUT_FILES_lisp}
)
add_dependencies(robot_msgs_generate_messages robot_msgs_generate_messages_lisp)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_dependencies(robot_msgs_generate_messages_lisp _robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_msgs_genlisp)
add_dependencies(robot_msgs_genlisp robot_msgs_generate_messages_lisp)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_msgs_generate_messages_lisp)

### Section generating for lang: gennodejs
### Generating Messages
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)
_generate_msg_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)

### Generating Services
_generate_srv_nodejs(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
)

### Generating Module File
_generate_module_nodejs(robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
  "${ALL_GEN_OUTPUT_FILES_nodejs}"
)

add_custom_target(robot_msgs_generate_messages_nodejs
  DEPENDS ${ALL_GEN_OUTPUT_FILES_nodejs}
)
add_dependencies(robot_msgs_generate_messages robot_msgs_generate_messages_nodejs)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_dependencies(robot_msgs_generate_messages_nodejs _robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_msgs_gennodejs)
add_dependencies(robot_msgs_gennodejs robot_msgs_generate_messages_nodejs)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_msgs_generate_messages_nodejs)

### Section generating for lang: genpy
### Generating Messages
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg"
  "${MSG_I_FLAGS}"
  "/opt/ros/noetic/share/std_msgs/cmake/../msg/Header.msg"
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)
_generate_msg_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)

### Generating Services
_generate_srv_py(robot_msgs
  "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv"
  "${MSG_I_FLAGS}"
  ""
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
)

### Generating Module File
_generate_module_py(robot_msgs
  ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
  "${ALL_GEN_OUTPUT_FILES_py}"
)

add_custom_target(robot_msgs_generate_messages_py
  DEPENDS ${ALL_GEN_OUTPUT_FILES_py}
)
add_dependencies(robot_msgs_generate_messages robot_msgs_generate_messages_py)

# add dependencies to all check dependencies targets
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/vision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/robot_ctrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_Yaw.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/barrel.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/PTZ_perception.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Track_reset.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/Yaw_Decision.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/competition_info.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/attack_base.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/msg/mainYawCtrl.msg" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})
get_filename_component(_filename "/home/hj/sentry_ros_25/src/robot_msgs/srv/ChassisMoveStatus.srv" NAME_WE)
add_dependencies(robot_msgs_generate_messages_py _robot_msgs_generate_messages_check_deps_${_filename})

# target for backward compatibility
add_custom_target(robot_msgs_genpy)
add_dependencies(robot_msgs_genpy robot_msgs_generate_messages_py)

# register target for catkin_package(EXPORTED_TARGETS)
list(APPEND ${PROJECT_NAME}_EXPORTED_TARGETS robot_msgs_generate_messages_py)



if(gencpp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gencpp_INSTALL_DIR}/robot_msgs
    DESTINATION ${gencpp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_cpp)
  add_dependencies(robot_msgs_generate_messages_cpp std_msgs_generate_messages_cpp)
endif()

if(geneus_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${geneus_INSTALL_DIR}/robot_msgs
    DESTINATION ${geneus_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_eus)
  add_dependencies(robot_msgs_generate_messages_eus std_msgs_generate_messages_eus)
endif()

if(genlisp_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genlisp_INSTALL_DIR}/robot_msgs
    DESTINATION ${genlisp_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_lisp)
  add_dependencies(robot_msgs_generate_messages_lisp std_msgs_generate_messages_lisp)
endif()

if(gennodejs_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs)
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${gennodejs_INSTALL_DIR}/robot_msgs
    DESTINATION ${gennodejs_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_nodejs)
  add_dependencies(robot_msgs_generate_messages_nodejs std_msgs_generate_messages_nodejs)
endif()

if(genpy_INSTALL_DIR AND EXISTS ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs)
  install(CODE "execute_process(COMMAND \"/usr/bin/python3\" -m compileall \"${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs\")")
  # install generated code
  install(
    DIRECTORY ${CATKIN_DEVEL_PREFIX}/${genpy_INSTALL_DIR}/robot_msgs
    DESTINATION ${genpy_INSTALL_DIR}
  )
endif()
if(TARGET std_msgs_generate_messages_py)
  add_dependencies(robot_msgs_generate_messages_py std_msgs_generate_messages_py)
endif()
