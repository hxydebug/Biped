cmake_minimum_required(VERSION 3.0.2)
project(biped)

# add_compile_options(-std=gnu++11)
# 使用更规范的方式设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)  # 可选：启用GNU扩展

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O2")
set (CMAKE_BUILD_TYPE "Release")

#添加C++ ABI定义以解决未定义引用问题
add_definitions(-D_GLIBCXX_USE_CXX11_ABI=1)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  serial
  geometry_msgs
  sensor_msgs
)

find_package(Eigen3 REQUIRED)
find_package(jsoncpp REQUIRED)

# set(QPOASES_INLUDE_DIRS /home/hxy/catkin_ws/src/licycle/third_party/qpOASES/include/qpOASES)
set(QPOASES_INLUDE_DIRS /usr/local/include/qpOASES)
set(dirPino "third_party/pinocchio")
set(dirQuill "third_party/quill")
set(incFmt "third_party/quill/quill/bundled")
set(incBoost "third_party/boost")
include_directories (${QPOASES_INLUDE_DIRS})
include_directories (${EIGEN3_INCLUDE_DIRS})
INCLUDE_DIRECTORIES("/usr/include/python3.6m")
INCLUDE_DIRECTORIES("~/openzen/include")

# third_party pinocchio, boost can be used, but need to installl urdfdom and jsoncpp
link_directories(third_party/pinocchio)
link_directories(third_party/quill)
link_directories(third_party/quill/quill/bundled)
link_directories(third_party/boost)
link_directories("/usr/lib/aarch64-linux-gnu/libjsoncpp.so")
link_directories("/usr/lib/aarch64-linux-gnu/liburdfdom_model.so")
link_directories("/usr/lib/aarch64-linux-gnu/libconsole_bridge.so")
link_directories("/usr/lib/aarch64-linux-gnu/libtinyxml.so")

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES bikebot
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

set(allDir ${dirPino} ${dirQuill})
set(allInc ${allDir} ${incFmt} ${incBoost})

include_directories(${allInc})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  /usr/include
  /usr/include/jsoncpp
)

#链接库
set(corLibs pinocchio quill)
#根据架构，处理连接库
set(sysCoreLibs)
if(${CMAKE_CXX_COMPILER} MATCHES "aarch64" OR ${CMAKE_SYSTEM_PROCESSOR} MATCHES "aarch64")
    message("linux arm64架构")
    foreach(lib ${corLibs})
        list(APPEND sysCoreLibs ${lib}_lin_arm64)
    endforeach()
else()
    message("linux x64架构")
    foreach(lib ${corLibs})
        list(APPEND sysCoreLibs ${lib}_lin_x64)
    endforeach()
endif()
message(${sysCoreLibs})

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
add_library(pino_kin_dyn    src/pino_kin_dyn.cpp)

target_link_libraries(pino_kin_dyn ${sysCoreLibs} pthread qpOASES libjsoncpp.so libconsole_bridge.so liburdfdom_model.so libtinyxml.so)
target_include_directories(pino_kin_dyn PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

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
target_link_libraries(Biped_control ${catkin_LIBRARIES} pino_kin_dyn pthread motor_drive leg_controller opmath -lpython3.6m swing_leg_controller stance_leg_controller gait_generator control PosVelEstimator)

add_executable(Calibrate node/calibrate.cpp)
target_link_libraries(Calibrate ${catkin_LIBRARIES} pthread motor_drive leg_controller opmath -lpython3.6m swing_leg_controller stance_leg_controller gait_generator control PosVelEstimator)
