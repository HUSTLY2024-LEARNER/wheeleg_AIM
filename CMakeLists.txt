cmake_minimum_required(VERSION 3.10)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
include("Cango")

project(AutoAim_HUST_2024)

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_BUILD_TYPE Release)

find_package(Python 3.8 REQUIRED COMPONENTS Development Interpreter)
find_package(pybind11 REQUIRED)
find_package(Threads REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Sophus REQUIRED)
find_package(Ceres REQUIRED)
find_package(OpenSSL REQUIRED)
find_package(OpenVINO REQUIRED)

include_directories(${Python_INCLUDE_DIRS})
include_directories(${pybind11_INCLUDE_DIRS})
include_directories("utils/include")
include_directories("manager/Headers")
include_directories("modules/monitor/Headers")

AddModulesInDirectory(
    DIRECTORY modules
    NAMES 
        simulator
        monitor
        driver
        detector
        solver
        tracker
        predictor
        activator
)

add_subdirectory(utils)

add_executable(AutoAim_HUST_2024 aim.cpp 
    modules/monitor/Sources/monitor.cpp 
    modules/simulator/Sources/simulator.cpp
    modules/detector/detector.cpp
    modules/solver/solver.cpp
    modules/predictor/predictor.cpp
    modules/activator/activator.cpp
    modules/tracker/tracker.cpp
    modules/driver/driver.cpp
)
target_link_libraries(AutoAim_HUST_2024
    PUBLIC simulator
    PUBLIC driver
    PUBLIC detector
    PUBLIC solver
    PUBLIC tracker
    PUBLIC predictor
    PUBLIC activator
)
