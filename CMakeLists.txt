cmake_minimum_required(VERSION 3.22)

#
# This file is generated only once,
# and is not re-generated if converter is called multiple times.
#
# User is free to modify the file as much as necessary
#

# Setup compiler settings
set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)


# Define the build type
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Debug")
endif()

# Set the project name
set(CMAKE_PROJECT_NAME CAN_demo)

# Enable compile command to ease indexing with e.g. clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS TRUE)

# Core project settings
project(${CMAKE_PROJECT_NAME})
message("Build type: " ${CMAKE_BUILD_TYPE})

# Enable CMake support for ASM and C languages
enable_language(C ASM)

# Create an executable object type
add_executable(${CMAKE_PROJECT_NAME}
        User/canio.c
        User/pid.c
        User/gm6020.c
        User/vofa.c)

# Add STM32CubeMX generated sources
add_subdirectory(cmake/stm32cubemx)

# Link directories setup
target_link_directories(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined library search paths
)

# Add sources to executable
target_sources(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user sources here
)

# Add include paths
target_include_directories(${CMAKE_PROJECT_NAME} PRIVATE
    User
    # Add user defined include paths
)

# Add project symbols (macros)
target_compile_definitions(${CMAKE_PROJECT_NAME} PRIVATE
    # Add user defined symbols
)

# newlib-nano disables floating-point printf unless this symbol is requested.
target_link_options(${CMAKE_PROJECT_NAME} PRIVATE -u _printf_float)

# Remove wrong libob.a library dependency when using cpp files
list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)

# Add linked libraries
target_link_libraries(${CMAKE_PROJECT_NAME}
    stm32cubemx

    # Add user defined libraries
)

# Generate common firmware images next to the ELF file.
set(PROJECT_HEX "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.hex")
set(PROJECT_BIN "${CMAKE_CURRENT_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin")

add_custom_command(TARGET ${CMAKE_PROJECT_NAME} POST_BUILD
    BYPRODUCTS
        ${PROJECT_HEX}
        ${PROJECT_BIN}
    COMMAND ${CMAKE_OBJCOPY} -O ihex
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${PROJECT_HEX}
    COMMAND ${CMAKE_OBJCOPY} -O binary
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}> ${PROJECT_BIN}
    COMMAND ${CMAKE_SIZE} --format=berkeley
        $<TARGET_FILE:${CMAKE_PROJECT_NAME}>
    COMMENT "Generating ELF, HEX and BIN firmware images"
    VERBATIM
)

set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY
    ADDITIONAL_CLEAN_FILES "${PROJECT_HEX};${PROJECT_BIN}"
)

# This target is intentionally manual: it is never run as part of a normal build.
find_program(STM32_PROGRAMMER_CLI
    NAMES STM32_Programmer_CLI
    HINTS "D:/Program Files/STM32CubeCLT_1.22.0/STM32CubeProgrammer/bin"
)

if(STM32_PROGRAMMER_CLI)
    add_custom_target(flash_stlink
        COMMAND ${STM32_PROGRAMMER_CLI}
            -c port=SWD
            -w ${PROJECT_HEX}
            -v
            -rst
        DEPENDS ${CMAKE_PROJECT_NAME} ${PROJECT_HEX}
        COMMENT "Programming and verifying with STM32CubeProgrammer over ST-Link"
        VERBATIM
    )
else()
    message(STATUS "STM32CubeProgrammer CLI not found; flash_stlink target is unavailable")
endif()
