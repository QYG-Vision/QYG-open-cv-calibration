# cmake needs this line
cmake_minimum_required(VERSION 3.1)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
# Define project name
project(calibration)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
# 头文件
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INCLUDE_DIR})

# 源文件
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)

add_executable(calibrateCamera calibrateCamera.cpp ${SRC_LIST})
add_executable(calculateError calculateError.cpp ${SRC_LIST})
add_executable(calibrateHandEye calibrateHandEye.cpp ${SRC_LIST})
add_executable(validateHandEye validateHandEye.cpp ${SRC_LIST})


# 海康SDK
target_include_directories(calibrateCamera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
target_include_directories(calculateError PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
target_include_directories(calibrateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
target_include_directories(validateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  target_link_directories(calibrateCamera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
  target_link_directories(calculateError PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
  target_link_directories(calibrateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
  target_link_directories(validateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  target_link_directories(calibrateCamera PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
  target_link_directories(calculateError PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
  target_link_directories(calibrateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
  target_link_directories(validateHandEye PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

# 检查是否需要链接filesystem库
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" AND CMAKE_CXX_COMPILER_VERSION VERSION_LESS "9.0")
    # GCC < 9.0 不需要显式链接
else()
    # GCC >= 9.0 和 Clang 需要链接 filesystem
    find_library(FILESYSTEM_LIB stdc++fs)
    if(FILESYSTEM_LIB)
        set(FILESYSTEM_LIBS ${FILESYSTEM_LIB})
    endif()
endif()

target_link_libraries(calibrateCamera PRIVATE ${OpenCV_LIBS} yaml-cpp MvCameraControl fmt::fmt ${FILESYSTEM_LIBS})
target_link_libraries(calculateError PRIVATE ${OpenCV_LIBS} yaml-cpp MvCameraControl fmt::fmt ${FILESYSTEM_LIBS})
target_link_libraries(calibrateHandEye PRIVATE ${OpenCV_LIBS} yaml-cpp MvCameraControl fmt::fmt ${FILESYSTEM_LIBS})
target_link_libraries(validateHandEye PRIVATE ${OpenCV_LIBS} yaml-cpp MvCameraControl fmt::fmt ${FILESYSTEM_LIBS})