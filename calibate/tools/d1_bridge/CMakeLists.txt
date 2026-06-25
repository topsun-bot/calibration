cmake_minimum_required(VERSION 3.10)
project(d1_bridge CXX)

set(CMAKE_CXX_STANDARD 17)

# d1_sdk 路径: cmake -DD1_SDK=/path/to/d1_sdk ..
set(D1_SDK "$ENV{D1_SDK}" CACHE PATH "Unitree D1 SDK 根目录")

if(NOT D1_SDK)
  set(D1_SDK "${CMAKE_SOURCE_DIR}/../../third_party/d1_sdk")
endif()

include_directories(
  ${D1_SDK}/include
  ${D1_SDK}/include/ddscxx
  ${CMAKE_SOURCE_DIR}/msg
)

if(EXISTS /usr/local/include/ddscxx)
  include_directories(/usr/local/include/ddscxx)
endif()
if(EXISTS /usr/local/include/iceoryx/v2.0.2)
  include_directories(/usr/local/include/iceoryx/v2.0.2)
endif()

link_directories(${D1_SDK}/lib)

set(CMAKE_BUILD_RPATH ${D1_SDK}/lib)
set(CMAKE_INSTALL_RPATH ${D1_SDK}/lib)

link_libraries(unitree_sdk2 ddsc ddscxx rt pthread)

set(MSG_ARM ${D1_SDK}/example/src/msg/ArmString_.cpp)
set(MSG_SERVO ${D1_SDK}/example/src/msg/PubServoInfo_.cpp)

if(NOT EXISTS ${MSG_ARM})
  set(MSG_ARM ${CMAKE_SOURCE_DIR}/msg/ArmString_.cpp)
endif()
if(NOT EXISTS ${MSG_SERVO})
  set(MSG_SERVO ${CMAKE_SOURCE_DIR}/msg/PubServoInfo_.cpp)
endif()

add_executable(d1_send_joints send_joints.cpp ${MSG_ARM})
add_executable(d1_read_joints read_joints.cpp ${MSG_SERVO})
add_executable(d1_arm_zero arm_zero.cpp ${MSG_ARM})
add_executable(d1_joint_enable joint_enable.cpp ${MSG_ARM})
add_executable(d1_arm_release arm_release.cpp ${MSG_ARM})
