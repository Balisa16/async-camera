cmake_minimum_required(VERSION 3.10)
project(asynccamera VERSION 0.1.0)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Uncomment if you want to set output location
# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
# set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)
# set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/lib)

# If you want to build shared library
# add_library(${PROJECT_NAME} SHARED
#     ${CMAKE_CURRENT_SOURCE_DIR}/src/async-camera.cpp
# )


include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${CMAKE_CURRENT_SOURCE_DIR}/include)

add_executable(${PROJECT_NAME} src/async-camera.cpp)
target_compile_options(${PROJECT_NAME} PRIVATE -std=gnu++17)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} Threads::Threads)
