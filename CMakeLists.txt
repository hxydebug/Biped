cmake_minimum_required(VERSION 3.0.2)
project(biped)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)

# set(QPOASES_INLUDE_DIRS /home/hxy/catkin_ws/src/licycle/third_party/qpOASES/include/qpOASES)
set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
include_directories (${QPOASES_INLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("/usr/include/python3.6m")
INCLUDE_DIRECTORIES("~/openzen/include")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bikebot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(OpenZenExampleCpp
    src/openzen_test.cpp)

target_link_libraries(OpenZenExampleCpp
    OpenZen)

target_compile_features(OpenZenExampleCpp
    PRIVATE
        cxx_std_11
)

add_executable(qptest 	src/qptest.cpp)
target_link_libraries(qptest qpOASES)

add_library(leg_controller	src/leg_controller.cpp)
add_library(opmath	src/opmath.cpp)

add_library(swing_leg_controller	src/swing_leg_controller.cpp)
add_library(stance_leg_controller	src/stance_leg_controller.cpp)
add_library(control	src/control.cpp)
add_library(gait_generator	src/gait_generator.cpp)
add_library(PosVelEstimator	src/PosVelEstimator.cpp)

target_link_libraries(opmath PUBLIC)
target_include_directories(opmath PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(gait_generator PUBLIC)
target_include_directories(gait_generator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(stance_leg_controller PUBLIC qpOASES gait_generator opmath)
target_include_directories(stance_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(swing_leg_controller PUBLIC gait_generator)
target_include_directories(swing_leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(PosVelEstimator PUBLIC gait_generator)
target_include_directories(PosVelEstimator PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(control PUBLIC)
target_include_directories(control PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(leg_controller PUBLIC swing_leg_controller stance_leg_controller gait_generator)
target_include_directories(leg_controller PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_library(motor_drive	src/motor_drive.cpp)
target_link_libraries(motor_drive PUBLIC)
target_include_directories(motor_drive PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(Biped_control node/bikebot_control.cpp)
target_link_libraries(Biped_control ${catkin_LIBRARIES} pthread motor_drive leg_controller opmath -lpython3.6m swing_leg_controller stance_leg_controller gait_generator control PosVelEstimator)

