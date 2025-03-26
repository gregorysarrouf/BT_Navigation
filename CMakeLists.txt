cmake_minimum_required(VERSION 3.10)

project(bt_nav)
set(CMAKE_CXX_STANDARD 17)

# Find the package
find_package(behaviortree_cpp REQUIRED)

add_executable(${PROJECT_NAME} "./src/bt_nav.cpp")
target_link_libraries(${PROJECT_NAME} behaviortree_cpp::behaviortree_cpp)
