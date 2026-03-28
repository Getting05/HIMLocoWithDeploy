cmake_minimum_required(VERSION 3.14)
project(deploy_cpp)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -Wall -Wextra")

# ========================== Dependencies ==========================

# ROS1 (catkin)
find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    sensor_msgs
)

find_package(yaml-cpp REQUIRED)

# LibTorch (PyTorch C++ API)
# Set Torch_DIR if not found automatically:
#   cmake -DTorch_DIR=/path/to/libtorch/share/cmake/Torch ..
find_package(Torch REQUIRED)

# Unitree Actuator SDK (headers and libs bundled in project)
set(UNITREE_SDK_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/include/Unitree_Motor")
set(UNITREE_SDK_LIB_DIR     "${CMAKE_CURRENT_SOURCE_DIR}/lib")

if(CMAKE_HOST_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(UNITREE_SDK_LIB "${UNITREE_SDK_LIB_DIR}/libUnitreeMotorSDK_Arm64.so")
else()
    set(UNITREE_SDK_LIB "${UNITREE_SDK_LIB_DIR}/libUnitreeMotorSDK_Linux64.so")
endif()

if(NOT EXISTS "${UNITREE_SDK_LIB}")
    message(FATAL_ERROR "Unitree SDK library not found: ${UNITREE_SDK_LIB}")
endif()

# ========================== Catkin Package ==========================

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS roscpp std_msgs sensor_msgs
)

# ========================== Include Directories ==========================

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${UNITREE_SDK_INCLUDE_DIR}
)

# ========================== Sources ==========================

set(SOURCES
    src/robot_runtime_config.cpp
    src/motor_driver.cpp
    src/imu_subscriber.cpp
    src/policy_runner.cpp
    src/keyboard_controller.cpp
    src/udp_controller.cpp
    src/state_machine.cpp
    src/robot_visualizer.cpp
    src/deploy_node.cpp
)

# ========================== RPATH ==========================
# Ensure the installed binary can find .so libs at runtime
set(CMAKE_INSTALL_RPATH "$ORIGIN;$ORIGIN/../lib;/opt/libtorch/lib")
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE)

# ========================== Executables ==========================

# Main deploy node
add_executable(deploy_node ${SOURCES})

target_link_libraries(deploy_node
    ${UNITREE_SDK_LIB}
    ${TORCH_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)

# Motor debug node (standalone, with RViz visualization)
add_executable(motor_debug_node
    src/robot_runtime_config.cpp
    src/motor_driver.cpp
    src/robot_visualizer.cpp
    src/motor_debug_node.cpp
)

target_link_libraries(motor_debug_node
    ${UNITREE_SDK_LIB}
    ${catkin_LIBRARIES}
    yaml-cpp
)

# ========================== Install ==========================

install(TARGETS deploy_node motor_debug_node
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Install Unitree SDK shared library alongside the executable
install(FILES ${UNITREE_SDK_LIB}
    DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
    OPTIONAL
)

install(DIRECTORY policy/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/policy
    OPTIONAL
)

install(DIRECTORY robot/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/robot
    OPTIONAL
)

install(DIRECTORY config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
    OPTIONAL
)

install(DIRECTORY sim/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/sim
    OPTIONAL
)
