cmake_minimum_required(VERSION 3.16)
project(laas_autonomous_car LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LAAS_USE_LIBSERIAL "Enable real UART through LibSerial" ON)
option(LAAS_BUILD_MPC "Build the optional MPC executable" ON)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

if(LAAS_USE_LIBSERIAL)
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(LIBSERIAL REQUIRED IMPORTED_TARGET libserial)
endif()

set(LAAS_CORE_SOURCES
    src/logical_robot/CameraInterface.cpp
    src/logical_robot/UdpYoloInterface.cpp
    src/logical_robot/UartVehicleInterface.cpp
    src/functional/perception/LaneDetectorCore.cpp
    src/functional/perception/LanePerceptionModule.cpp
    src/decision/MissionSupervisor.cpp
    src/functional/planning/LaneChangePlanner.cpp
    src/functional/planning/LaneChangePlannerModule.cpp
    src/functional/control/PurePursuitController.cpp
    src/functional/control/PurePursuitControlModule.cpp
    src/functional/safety/SafetyFilterModule.cpp
    src/execution_control/Scheduler.cpp
    src/execution_control/Executive.cpp
)

function(configure_laas_target target_name)
    target_include_directories(${target_name} PRIVATE
        ${OpenCV_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_link_libraries(${target_name} PRIVATE
        ${OpenCV_LIBS}
        Threads::Threads
    )

    if(LAAS_USE_LIBSERIAL)
        target_compile_definitions(${target_name} PRIVATE LAAS_USE_LIBSERIAL)
        target_link_libraries(${target_name} PRIVATE PkgConfig::LIBSERIAL)
    endif()
endfunction()

add_executable(laas_pp
    src/app/main.cpp
    ${LAAS_CORE_SOURCES}
)
configure_laas_target(laas_pp)

if(LAAS_BUILD_MPC)
    find_package(Eigen3 REQUIRED)
    find_package(OsqpEigen REQUIRED)

    add_executable(laas_mpc
        src/app/main.cpp
        ${LAAS_CORE_SOURCES}
        src/functional/control/MpcController.cpp
        src/functional/control/MpcControlModule.cpp
    )
    configure_laas_target(laas_mpc)
    target_compile_definitions(laas_mpc PRIVATE LAAS_ENABLE_MPC)
    target_link_libraries(laas_mpc PRIVATE
        Eigen3::Eigen
        OsqpEigen::OsqpEigen
    )
endif()
