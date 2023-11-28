cmake_minimum_required(VERSION 3.12)

# Change your executable name to something creative!
set(NAME Head) # <-- Name your project/executable here!

include(pico_sdk_import.cmake)

# Gooey boilerplate
project(${NAME} C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Initialize the SDK
pico_sdk_init()

SET(FREERTOS_KERNEL_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/FreeRTOS-Kernel" CACHE STRING "Common Lib")
SET(FREERTOS_CONFIG_FILE_DIRECTORY "${CMAKE_CURRENT_LIST_DIR}/port/FreeRTOS-Kernel" CACHE STRING "Local Config")
include_directories("${FREERTOS_CONFIG_FILE_DIRECTORY}") 
include(FreeRTOS_Kernel_import.cmake)
include(${FREERTOS_CONFIG_FILE_DIRECTORY}/freertos_config.cmake)

#Allow New and Delete to be redefined for FreeRTOS
add_compile_definitions(
	PICO_CXX_DISABLE_ALLOCATION_OVERRIDES=1
)

SET(MICRO_ROS_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/micro_ros_raspberrypi_pico_sdk/" CACHE STRING "Common Lib")
include(micro_ros.cmake)

SET(EIGEN_PATH "${CMAKE_CURRENT_LIST_DIR}/lib/eigen/" CACHE STRING "Common Lib")
include(eigen_import.cmake)

add_subdirectory(src)
add_subdirectory(test EXCLUDE_FROM_ALL)

#Set up files for the release packages
install(CODE "execute_process(COMMAND $ENV{HOME}/bin/picoDeploy.sh ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.elf)")

# Set up files for the release packages
install(FILES
    ${CMAKE_CURRENT_BINARY_DIR}/src/${NAME}.uf2
    DESTINATION ${CMAKE_CURRENT_BINARY_DIR}
)

set(CPACK_INCLUDE_TOPLEVEL_DIRECTORY OFF)
set(CPACK_GENERATOR "ZIP" "TGZ")
include(CPack)
