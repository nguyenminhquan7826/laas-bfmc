cmake_minimum_required(VERSION 3.16)
project(laas_autonomous_car LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(LAAS_USE_LIBSERIAL "Enable real UART through LibSerial" ON)
option(LAAS_BUILD_MPC "Build the optional MPC executable" ON)
option(
    LAAS_BUILD_PARKING_CLIENT
    "Build TCP/NDJSON Parking Server client when json-c development files are available"
    ON
)

find_package(OpenCV REQUIRED)
find_package(Threads REQUIRED)

set(LAAS_PARKING_CLIENT_AVAILABLE OFF)

if(LAAS_USE_LIBSERIAL OR LAAS_BUILD_PARKING_CLIENT)
    find_package(PkgConfig REQUIRED)
endif()

if(LAAS_USE_LIBSERIAL)
    pkg_check_modules(
        LIBSERIAL
        REQUIRED
        IMPORTED_TARGET
        libserial
    )
endif()

if(LAAS_BUILD_PARKING_CLIENT)
    pkg_check_modules(
        JSONC
        QUIET
        IMPORTED_TARGET
        json-c
    )

    if(JSONC_FOUND)
        set(LAAS_PARKING_CLIENT_AVAILABLE ON)
    else()
        message(
            WARNING
            "json-c development files not found: Parking Server client disabled. Install libjson-c-dev on Raspberry Pi."
        )
    endif()
endif()

set(LAAS_CORE_SOURCES

    # =========================
    # Logical robot
    # =========================
    src/logical_robot/CameraInterface.cpp
    src/logical_robot/UdpYoloInterface.cpp
    src/logical_robot/UartVehicleInterface.cpp
    src/logical_robot/TelemetryProtocol.cpp

    # =========================
    # Perception
    # =========================
    src/functional/perception/LaneDetectorCore.cpp
    src/functional/perception/LanePerceptionModule.cpp
    src/functional/perception/ParkingStatusBenchSource.cpp

    # =========================
    # Localization
    # =========================
    src/functional/localization/VehiclePoseEstimator.cpp

    # =========================
    # Decision
    # =========================
    src/decision/MissionSupervisor.cpp

    # =========================
    # Planning
    # =========================
    src/functional/planning/LaneChangePlanner.cpp
    src/functional/planning/LaneChangePlannerModule.cpp

    # =========================
    # Control
    # =========================
    src/functional/control/PurePursuitController.cpp
    src/functional/control/PurePursuitControlModule.cpp
    src/functional/control/ParkingTrajectoryTracker.cpp

    # =========================
    # Safety
    # =========================
    src/functional/safety/SafetyFilterModule.cpp
    src/functional/safety/ParkingTrajectoryValidator.cpp
    src/functional/safety/ParkingSafetyFilter.cpp

    # =========================
    # Execution
    # =========================
    src/execution_control/Scheduler.cpp
    src/execution_control/Executive.cpp
)

# Parking TCP/NDJSON components require json-c.
if(LAAS_PARKING_CLIENT_AVAILABLE)
    list(
        APPEND
        LAAS_CORE_SOURCES

        src/logical_robot/ParkingProtocol.cpp
        src/logical_robot/ParkingServerClient.cpp
    )
endif()

function(configure_laas_target target_name)

    target_include_directories(
        ${target_name}
        PRIVATE
            ${OpenCV_INCLUDE_DIRS}
            ${CMAKE_CURRENT_SOURCE_DIR}/src
    )

    target_link_libraries(
        ${target_name}
        PRIVATE
            ${OpenCV_LIBS}
            Threads::Threads
    )

    if(LAAS_USE_LIBSERIAL)

        target_compile_definitions(
            ${target_name}
            PRIVATE
                LAAS_USE_LIBSERIAL
        )

        target_link_libraries(
            ${target_name}
            PRIVATE
                PkgConfig::LIBSERIAL
        )

    endif()

    if(LAAS_PARKING_CLIENT_AVAILABLE)

        target_compile_definitions(
            ${target_name}
            PRIVATE
                LAAS_ENABLE_PARKING_CLIENT
        )

        target_link_libraries(
            ${target_name}
            PRIVATE
                PkgConfig::JSONC
        )

    endif()

endfunction()

# =========================
# Pure Pursuit executable
# =========================

add_executable(
    laas_pp

    src/app/main.cpp
    ${LAAS_CORE_SOURCES}
)

configure_laas_target(laas_pp)

# =========================
# Optional MPC executable
# =========================

if(LAAS_BUILD_MPC)

    find_package(Eigen3 REQUIRED)
    find_package(OsqpEigen REQUIRED)

    add_executable(
        laas_mpc

        src/app/main.cpp
        ${LAAS_CORE_SOURCES}

        src/functional/control/MpcController.cpp
        src/functional/control/MpcControlModule.cpp
    )

    configure_laas_target(laas_mpc)

    target_compile_definitions(
        laas_mpc
        PRIVATE
            LAAS_ENABLE_MPC
    )

    target_link_libraries(
        laas_mpc
        PRIVATE
            Eigen3::Eigen
            OsqpEigen::OsqpEigen
    )

endif()