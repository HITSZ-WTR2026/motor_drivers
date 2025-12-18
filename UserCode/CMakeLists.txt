# UserCode/CMakeLists.txt
cmake_minimum_required(VERSION 3.21)
project(motor_drivers)

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
option(ENABLE_BSP "enable BSP layer" ON)
option(ENABLE_LIBS "enable LIBS layer" ON)
option(ENABLE_CONTROLLERS "enable CONTROLLERS layer" ON)

# ---------------------------------------------------------------------------
# collect layer sources
# ---------------------------------------------------------------------------
set(ALL_SOURCES interfaces/motor_if.c)

set(ALL_HEADERS "interfaces/motor_if.h")

if (ENABLE_BSP)
    file(GLOB_RECURSE BSP_SOURCES bsp/*.c)
    list(APPEND ALL_SOURCES ${BSP_SOURCES})
    list(APPEND ALL_HEADERS
            "bsp/can_driver.h"
            "bsp/gpio_driver.h"
            "bsp/pwm.h"
    )
endif ()

if (ENABLE_LIBS)
    file(GLOB_RECURSE LIB_SOURCES libs/*.c)
    list(APPEND ALL_SOURCES ${LIB_SOURCES})
    list(APPEND ALL_HEADERS
            "libs/pid_motor.h"
            "libs/pid_pd.h"
            "libs/s_curve.h"
    )
endif ()

if (ENABLE_CONTROLLERS)
    file(GLOB_RECURSE CONTROLLER_SOURCES controllers/*.c)
    list(APPEND ALL_SOURCES ${CONTROLLER_SOURCES})
    list(APPEND ALL_HEADERS "controllers/s_curve_traj_follower.h")
endif ()

# ---------------------------------------------------------------------------
# feature options: option + source + macro
# ---------------------------------------------------------------------------
set(DRIVER_MACROS "")

option(ENABLE_DJI_DRIVER "enable DJI driver" ON)
if (ENABLE_DJI_DRIVER)
    list(APPEND ALL_SOURCES drivers/DJI.c)
    list(APPEND ALL_HEADERS "drivers/DJI.h")
    list(APPEND DRIVER_MACROS USE_DJI)
endif ()

option(ENABLE_TB6612_DRIVER "enable TB6612 driver" OFF)
if (ENABLE_TB6612_DRIVER)
    list(APPEND ALL_SOURCES drivers/tb6612.c)
    list(APPEND ALL_HEADERS "drivers/tb6612.h")
    list(APPEND DRIVER_MACROS USE_TB6612)
endif ()

option(ENABLE_VESC_DRIVER "enable VESC driver" OFF)
if (ENABLE_VESC_DRIVER)
    list(APPEND ALL_SOURCES drivers/vesc.c)
    list(APPEND ALL_HEADERS "drivers/vesc.h")
    list(APPEND DRIVER_MACROS USE_VESC)
endif ()

option(ENABLE_DM_DRIVER "enable DM driver" OFF)
if (ENABLE_DM_DRIVER)
    list(APPEND ALL_SOURCES drivers/DM.c)
    list(APPEND ALL_HEADERS "drivers/DM.h")
    list(APPEND DRIVER_MACROS USE_DM)
endif ()

# ---------------------------------------------------------------------------
# create static library
# ---------------------------------------------------------------------------
add_library(${PROJECT_NAME} STATIC ${ALL_SOURCES})

# ---------------------------------------------------------------------------
# compile definitions
# ---------------------------------------------------------------------------
if (DRIVER_MACROS)
    target_compile_definitions(${PROJECT_NAME} PUBLIC ${DRIVER_MACROS})
endif ()

# ---------------------------------------------------------------------------
# build/include 目录
# ---------------------------------------------------------------------------
set(EXPORT_INCLUDE_DIR ${CMAKE_CURRENT_BINARY_DIR}/include)
file(MAKE_DIRECTORY ${EXPORT_INCLUDE_DIR})
target_include_directories(${PROJECT_NAME} PUBLIC ${EXPORT_INCLUDE_DIR})

# ---------------------------------------------------------------------------
# copy selected headers
# ---------------------------------------------------------------------------
foreach (header IN LISTS ALL_HEADERS)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/${header}
            ${EXPORT_INCLUDE_DIR}/${header} COPYONLY)
endforeach ()