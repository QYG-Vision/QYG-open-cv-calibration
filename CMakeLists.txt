# cmake needs this line
cmake_minimum_required(VERSION 3.14)
## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
# Define project name
project(calibration)

find_package(OpenCV REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
find_package(Threads REQUIRED)
# 源文件
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src SRC_LIST)

# 公共库：避免在多个可执行文件中重复编译 src 下源码
add_library(calibration_core STATIC ${SRC_LIST})
target_include_directories(calibration_core
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/include
)

add_executable(calibrateCamera apps/calibrateCamera.cpp)
add_executable(calculateError apps/calculateError.cpp)
add_executable(calibrateHandEye apps/calibrateHandEye.cpp)
add_executable(validateHandEye apps/validateHandEye.cpp)
add_executable(previewCamera apps/previewCamera.cpp)
add_executable(calibrationWorkbench apps/calibrationWorkbench.cpp)

# 链接公共库
target_link_libraries(calibrateCamera PRIVATE calibration_core)
target_link_libraries(calculateError PRIVATE calibration_core)
target_link_libraries(calibrateHandEye PRIVATE calibration_core)
target_link_libraries(validateHandEye PRIVATE calibration_core)
target_link_libraries(previewCamera PRIVATE calibration_core)
target_link_libraries(calibrationWorkbench PRIVATE calibration_core)

# 海康SDK
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
  set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/amd64)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64")
  set(HIK_LIB_DIR ${CMAKE_CURRENT_SOURCE_DIR}/hikSDK/lib/arm64)
else()
  message(FATAL_ERROR "Unsupported architecture: ${CMAKE_HOST_SYSTEM_PROCESSOR}!")
endif()

target_link_directories(calibration_core PUBLIC ${HIK_LIB_DIR})

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

target_link_libraries(calibration_core
  PUBLIC
    ${OpenCV_LIBS}
    yaml-cpp
    MvCameraControl
    fmt::fmt
    Threads::Threads
    ${FILESYSTEM_LIBS}
)

# ============================================================
# 测试体系
# ============================================================
option(BUILD_TESTING "Build unit and integration tests" OFF)

include(CTest)

if(BUILD_TESTING)
    # ----- GoogleTest via FetchContent -----
    include(FetchContent)
    FetchContent_Declare(
        googletest
        GIT_REPOSITORY https://github.com/google/googletest.git
        GIT_TAG v1.15.2
    )
    # Prevent gtest from installing itself
    set(INSTALL_GTEST OFF CACHE BOOL "" FORCE)
    FetchContent_MakeAvailable(googletest)
    include(GoogleTest)

    # ----- 测试数据根目录 -----
    set(TEST_DATA_DIR ${CMAKE_CURRENT_SOURCE_DIR}/tests/data)

    # ----- 单元测试 -----
    add_executable(test_handeye_result tests/unit/test_handeye_result.cpp)
    target_link_libraries(test_handeye_result PRIVATE gtest_main calibration_core)
    gtest_discover_tests(test_handeye_result)
    # ThreadSafeQueue
    add_executable(test_thread_safe_queue tests/unit/test_thread_safe_queue.cpp)
    target_include_directories(test_thread_safe_queue PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(test_thread_safe_queue PRIVATE gtest_main Threads::Threads)
    gtest_discover_tests(test_thread_safe_queue)

    add_executable(test_intrinsic_result_parser tests/unit/test_intrinsic_result_parser.cpp)
    target_link_libraries(test_intrinsic_result_parser PRIVATE gtest_main calibration_core)
    gtest_discover_tests(test_intrinsic_result_parser)

    add_executable(test_reprojection_validation tests/unit/test_reprojection_validation.cpp)
    target_compile_definitions(test_reprojection_validation PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
    target_link_libraries(test_reprojection_validation PRIVATE gtest_main calibration_core)
    gtest_discover_tests(test_reprojection_validation)

    add_executable(test_task_controller tests/unit/test_task_controller.cpp)
    target_compile_definitions(test_task_controller PRIVATE TEST_DATA_DIR="${TEST_DATA_DIR}")
    target_link_libraries(test_task_controller PRIVATE gtest_main calibration_core util)
    gtest_discover_tests(test_task_controller)

    add_executable(test_serial_driver_shutdown tests/unit/test_serial_driver_shutdown.cpp)
    target_link_libraries(test_serial_driver_shutdown PRIVATE gtest_main calibration_core util)
    gtest_discover_tests(test_serial_driver_shutdown)

    # QYG 串口协议（与 QD_Vision26 rm_serial_driver 字节级对齐）
    add_executable(test_qyg_protocol tests/unit/test_qyg_protocol.cpp)
    target_include_directories(test_qyg_protocol PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(test_qyg_protocol PRIVATE gtest_main calibration_core)
    gtest_discover_tests(test_qyg_protocol)

    # QD 串口协议（16 字节定长帧，标定工具当前使用的协议）
    add_executable(test_qd_protocol tests/unit/test_qd_protocol.cpp)
    target_include_directories(test_qd_protocol PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(test_qd_protocol PRIVATE gtest_main calibration_core)
    gtest_discover_tests(test_qd_protocol)

    # AutoCollector
    add_executable(test_auto_collector tests/unit/test_auto_collector.cpp)
    target_include_directories(test_auto_collector PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(test_auto_collector
        PRIVATE
            gtest_main
            calibration_core
    )
    gtest_discover_tests(test_auto_collector)

    # Image_Reader
    add_executable(test_image_reader tests/unit/test_image_reader.cpp)
    target_include_directories(test_image_reader PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_link_libraries(test_image_reader
        PRIVATE
            gtest_main
            calibration_core
    )
    gtest_discover_tests(test_image_reader)

    # IntrinsicCalibrator
    add_executable(test_intrinsic_calibrator tests/unit/test_intrinsic_calibrator.cpp)
    target_include_directories(test_intrinsic_calibrator PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_definitions(test_intrinsic_calibrator
        PRIVATE
            TEST_DATA_DIR="${TEST_DATA_DIR}"
    )
    target_link_libraries(test_intrinsic_calibrator
        PRIVATE
            gtest_main
            calibration_core
    )
    gtest_discover_tests(test_intrinsic_calibrator)

    # ----- 离线集成测试：手眼标定黑盒验证 -----
    add_executable(test_handeye_offline tests/integration/test_handeye_offline.cpp)
    target_include_directories(test_handeye_offline PRIVATE ${PROJECT_SOURCE_DIR}/include)
    target_compile_definitions(test_handeye_offline
        PRIVATE
            CALIBRATE_HANDEYE_BINARY="${CMAKE_CURRENT_BINARY_DIR}/calibrateHandEye"
            TEST_DATA_DIR="${TEST_DATA_DIR}"
    )
    target_link_libraries(test_handeye_offline
        PRIVATE
            gtest_main
            calibration_core
    )
    gtest_discover_tests(test_handeye_offline
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
endif()
