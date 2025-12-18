# UserCode/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)

set(LIB_NAME motor_drivers)

project(${LIB_NAME})

# 导库方式
# add_subdirectory({project_path}/UserCode)
# target_link_libraries(${PROJECT_NAME}.elf PRIVATE motor_drivers)

# ---------------------------------------------------------------------------
# guard: UserCode should only be included by parent project
# ---------------------------------------------------------------------------
if (PROJECT_IS_TOP_LEVEL)
    message(FATAL_ERROR "UserCode must be included via add_subdirectory() from parent project")
endif ()

# ---------------------------------------------------------------------------
# layer options
# ---------------------------------------------------------------------------
option(MotorIF_UseBSP "use bsp layer" ON)
option(MotorIF_UseLibs "use library (pid_motor, pid_pd)" ON)
option(MotorIF_UseControllers "use s-curve-traj (depend on `s_curve`)" ON)

# ---------------------------------------------------------------------------
# collect layer sources
# ---------------------------------------------------------------------------
set(ALL_SOURCES interfaces/motor_if.c)

set(ALL_HEADERS "interfaces/motor_if.h")

if (MotorIF_UseBSP)
    file(GLOB_RECURSE BSP_SOURCES bsp/*.c)
    list(APPEND ALL_SOURCES ${BSP_SOURCES})
    list(APPEND ALL_HEADERS
            "bsp/can_driver.h"
            "bsp/gpio_driver.h"
            "bsp/pwm.h"
    )
endif ()

if (MotorIF_UseLibs)
    file(GLOB_RECURSE LIB_SOURCES libs/*.c)
    list(APPEND ALL_SOURCES ${LIB_SOURCES})
    list(APPEND ALL_HEADERS
            "libs/pid_motor.h"
            "libs/pid_pd.h"
    )
endif ()

if (MotorIF_UseControllers)
    file(GLOB_RECURSE CONTROLLER_SOURCES controllers/*.c)
    list(APPEND ALL_SOURCES ${CONTROLLER_SOURCES})
    list(APPEND ALL_HEADERS "controllers/s_curve_traj_follower.h")
endif ()

# ---------------------------------------------------------------------------
# feature options: option + source + macro
# ---------------------------------------------------------------------------
set(DRIVER_MACROS "")

option(MotorIF_UseDJI "enable DJI driver" ON)
if (MotorIF_UseDJI)
    list(APPEND ALL_SOURCES drivers/DJI.c)
    list(APPEND ALL_HEADERS "drivers/DJI.h")
    list(APPEND DRIVER_MACROS USE_DJI)
endif ()

option(MotorIF_UseTB6612 "enable TB6612 driver" OFF)
if (MotorIF_UseTB6612)
    list(APPEND ALL_SOURCES drivers/tb6612.c)
    list(APPEND ALL_HEADERS "drivers/tb6612.h")
    list(APPEND DRIVER_MACROS USE_TB6612)
endif ()

option(MotorIF_UseVESC "enable VESC driver" OFF)
if (MotorIF_UseVESC)
    list(APPEND ALL_SOURCES drivers/vesc.c)
    list(APPEND ALL_HEADERS "drivers/vesc.h")
    list(APPEND DRIVER_MACROS USE_VESC)
endif ()

option(MotorIF_UseDM "enable DM driver" OFF)
if (MotorIF_UseDM)
    list(APPEND ALL_SOURCES drivers/DM.c)
    list(APPEND ALL_HEADERS "drivers/DM.h")
    list(APPEND DRIVER_MACROS USE_DM)
endif ()

# ---------------------------------------------------------------------------
# create static library
# ---------------------------------------------------------------------------
add_library(motor_drivers STATIC ${ALL_SOURCES})
if (NOT TARGET ${LIB_NAME})
    message(FATAL_ERROR "target ${LIB_NAME} not created")
endif ()

# ---------------------------------------------------------------------------
# compile definitions
# ---------------------------------------------------------------------------
if (DRIVER_MACROS)
    target_compile_definitions(${LIB_NAME} PUBLIC ${DRIVER_MACROS})
endif ()

# ---------------------------------------------------------------------------
# build/include 目录
# ---------------------------------------------------------------------------
set(EXPORT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${EXPORT_INCLUDE_DIR})
target_include_directories(${LIB_NAME} PUBLIC ${EXPORT_INCLUDE_DIR})

# ---------------------------------------------------------------------------
# copy selected headers
# ---------------------------------------------------------------------------
foreach (header IN LISTS ALL_HEADERS)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${header}
            ${EXPORT_INCLUDE_DIR}/${header} COPYONLY)
endforeach ()


# ---------------------------------------------------------------------------
# declare dependencies
# ---------------------------------------------------------------------------
add_dependencies(${LIB_NAME} stm32cubemx)
target_link_libraries(${LIB_NAME} PRIVATE stm32cubemx)

if (ENABLE_CONTROLLERS)
    # Repo: https://github.com/HITSZ-WTRobot/s-curve-planner
    target_link_libraries(${PROJECT_NAME} PRIVATE s_curve)
    add_dependencies(${PROJECT_NAME} s_curve)
endif ()

target_link_libraries(${LIB_NAME} PRIVATE m)
