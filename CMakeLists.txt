cmake_minimum_required(VERSION 3.10)
project(CeresDemo)

# 设置C++标准
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED True)

# 查找需要的包
find_package(Ceres REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)


# 包含头文件
include_directories(${CERES_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")


# 添加可执行文件
add_executable(optDemo optDemo.cpp)

# 链接库
target_link_libraries(optDemo ${CERES_LIBRARIES} ${Pangolin_LIBRARIES} ${OpenCV_LIBS})
