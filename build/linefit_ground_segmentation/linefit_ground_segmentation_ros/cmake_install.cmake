# Install script for directory: /home/hj/sentry_ros_25/src/linefit_ground_segmentation/linefit_ground_segmentation_ros

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/hj/sentry_ros_25/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Release")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/hj/sentry_ros_25/build/linefit_ground_segmentation/linefit_ground_segmentation_ros/catkin_generated/installspace/linefit_ground_segmentation_ros.pc")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/linefit_ground_segmentation_ros/cmake" TYPE FILE FILES
    "/home/hj/sentry_ros_25/build/linefit_ground_segmentation/linefit_ground_segmentation_ros/catkin_generated/installspace/linefit_ground_segmentation_rosConfig.cmake"
    "/home/hj/sentry_ros_25/build/linefit_ground_segmentation/linefit_ground_segmentation_ros/catkin_generated/installspace/linefit_ground_segmentation_rosConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/linefit_ground_segmentation_ros" TYPE FILE FILES "/home/hj/sentry_ros_25/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/package.xml")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros" TYPE EXECUTABLE FILES "/home/hj/sentry_ros_25/devel/lib/linefit_ground_segmentation_ros/ground_segmentation_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node"
         OLD_RPATH "/opt/ros/noetic/lib:/home/hj/sentry_ros_25/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros" TYPE EXECUTABLE FILES "/home/hj/sentry_ros_25/devel/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node"
         OLD_RPATH "/opt/ros/noetic/lib:/home/hj/sentry_ros_25/devel/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/linefit_ground_segmentation_ros/ground_segmentation_test_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/linefit_ground_segmentation_ros" TYPE FILE FILES
    "/home/hj/sentry_ros_25/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/segmentation.launch"
    "/home/hj/sentry_ros_25/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/test.launch"
    "/home/hj/sentry_ros_25/src/linefit_ground_segmentation/linefit_ground_segmentation_ros/launch/segmentation_params.yaml"
    )
endif()

