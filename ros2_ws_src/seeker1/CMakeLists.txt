cmake_minimum_required(VERSION 3.28.1)
project(seeker)

find_package(OpenCV REQUIRED)

find_package(ament_cmake QUIET)

# ROS2依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(stereo_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)

# 添加包含目录
include_directories(
        include
)

# 根据架构设置库目录
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86")
    set(LIB_DIR "libs/x86")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
    set(LIB_DIR "libs/arm64")
endif()

link_directories(${LIB_DIR}
)

# 生成可执行文件
add_executable(seeker_node
        src/seeker_ros2.cpp
)

# 链接依赖库
target_include_directories(seeker_node
        PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seeker_node
        ${OpenCV_LIBRARIES}
        seeker
        usb-1.0
)

ament_target_dependencies(seeker_node rclcpp std_msgs geometry_msgs sensor_msgs stereo_msgs cv_bridge image_transport)
# 安装规则

# ---------------------------------------------------------------------------
# seeker_split_node —— 把 /all/compressed 拆成 VINS 用的双目 mono8
# 用 C++ 而非 Python: rclpy 收发大图像上限只有 ~9Hz，C++ 可跑满相机的 20Hz
# ---------------------------------------------------------------------------
add_executable(seeker_split_node
        src/seeker_split_node.cpp
)

target_include_directories(seeker_split_node
        PRIVATE
        ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(seeker_split_node
        ${OpenCV_LIBRARIES}
)

ament_target_dependencies(seeker_split_node
        rclcpp
        sensor_msgs
)

install(TARGETS seeker_split_node
        DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS seeker_node
        DESTINATION lib/${PROJECT_NAME}
)

install(FILES
        ${LIB_DIR}/libseeker.so
        DESTINATION lib/
)

install(DIRECTORY
        launch/ros2
        DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY ./config/ DESTINATION share/${PROJECT_NAME}/config/)

ament_package()

