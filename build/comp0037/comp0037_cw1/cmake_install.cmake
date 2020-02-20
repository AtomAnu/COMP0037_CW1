# Install script for directory: /home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/ros_user/COMP0037_CW1/install")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
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

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/pkgconfig" TYPE FILE FILES "/home/ros_user/COMP0037_CW1/build/comp0037/comp0037_cw1/catkin_generated/installspace/comp0037_cw1.pc")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/comp0037_cw1/cmake" TYPE FILE FILES
    "/home/ros_user/COMP0037_CW1/build/comp0037/comp0037_cw1/catkin_generated/installspace/comp0037_cw1Config.cmake"
    "/home/ros_user/COMP0037_CW1/build/comp0037/comp0037_cw1/catkin_generated/installspace/comp0037_cw1Config-version.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/comp0037_cw1" TYPE FILE FILES "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/package.xml")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/comp0037_cw1" TYPE FILE FILES
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/square_terrain_scenario.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/test_scenario.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/lifo_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/greedy_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/a_star_manhatton_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/a_star_euclidean_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/factory_scenario.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/dijkstra_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/fifo_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/factory_terrain_scenario.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/a_star_constant_standalone.launch"
    "/home/ros_user/COMP0037_CW1/src/comp0037/comp0037_cw1/launch/a_star_octile_standalone.launch"
    )
endif()

