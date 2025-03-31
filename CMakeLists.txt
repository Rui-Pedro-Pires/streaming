cmake_minimum_required(VERSION 3.10)
project(video_subscriber)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(zenohc_DIR "/usr/local/lib/cmake/zenohc")
set(zenohcxx_DIR "/usr/local/lib/cmake/zenohcxx")

find_package(OpenCV REQUIRED)
find_package(zenohc REQUIRED)
find_package(zenohcxx REQUIRED)

include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(video_subscriber video_subscriber.cpp)
target_link_libraries(video_subscriber ${OpenCV_LIBS} zenohcxx::zenohc)
