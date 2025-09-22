# cmake needs this line
cmake_minimum_required(VERSION 3.1)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
# Define project name
project(calibration)

# 头文件
include_directories(${PROJECT_SOURCE_DIR}/include)
# 源文件
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)


find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(${PROJECT_NAME} ${SRC_LIST})

# 海康SDK
target_include_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(${PROJECT_NAME} PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_libraries(${PROJECT_NAME} PRIVATE ${OpenCV_LIBS} yaml-cpp MvCameraControl)