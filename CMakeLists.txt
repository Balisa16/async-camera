cmake_minimum_required(VERSION 3.10)
project(asynccamera VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(async-camera src/main.cpp)
target_compile_options(async-camera PRIVATE -std=gnu++17)
target_link_libraries(async-camera ${OpenCV_LIBS} Threads::Threads)
