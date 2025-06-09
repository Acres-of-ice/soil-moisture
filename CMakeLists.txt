cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/components)
set(SPIFFS_IMAGE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/spiffs_image")

# Get version from git tag
execute_process(
    COMMAND git describe --tags --abbrev=0
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE GIT_TAG
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET
)

# If no tag found, use default version
if(NOT GIT_TAG)
    set(GIT_TAG "v0.0.0")
endif()

# Remove 'v' prefix if present
string(REGEX REPLACE "^v" "" BASE_VERSION ${GIT_TAG})

# Version file path
set(VERSION_FILE ${CMAKE_SOURCE_DIR}/version.txt)

# Read current version and extract build counter
set(BUILD_COUNTER 0)
if(EXISTS ${VERSION_FILE})
    file(READ ${VERSION_FILE} CURRENT_VERSION)
    string(STRIP ${CURRENT_VERSION} CURRENT_VERSION)
    
    # Extract base version and build counter from existing version
    if(CURRENT_VERSION MATCHES "^([0-9]+\\.[0-9]+\\.[0-9]+)\\+([0-9]+)$")
        set(EXISTING_BASE_VERSION ${CMAKE_MATCH_1})
        set(BUILD_COUNTER ${CMAKE_MATCH_2})
        
        # If base version changed, reset counter
        if(NOT EXISTING_BASE_VERSION STREQUAL BASE_VERSION)
            set(BUILD_COUNTER 0)
        endif()
    endif()
endif()

# Increment build counter
math(EXPR BUILD_COUNTER "${BUILD_COUNTER} + 1")

# Create full version with build number
string(CONCAT PROJECT_VERSION ${BASE_VERSION} "+" ${BUILD_COUNTER})

message(STATUS "PROJECT_VERSION: '${PROJECT_VERSION}'")

# Write version to version.txt
file(WRITE ${VERSION_FILE} ${PROJECT_VERSION})

# Set project version (ESP-IDF uses semantic version without build metadata)
set(PROJECT_VER ${BASE_VERSION})

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

# Start with default project name
project(soil_moisture VERSION ${BASE_VERSION})

# After project() is called, CONFIG variables should be available
# Check if configuration variables are available and update project name
if(DEFINED CONFIG_MASTER AND CONFIG_MASTER)
    set(ACTUAL_PROJECT_NAME "${CONFIG_SITE_NAME}_MASTER")
    message(STATUS "Building MASTER for site: ${CONFIG_SITE_NAME}")
elseif(DEFINED CONFIG_SOIL AND CONFIG_SOIL)
    set(ACTUAL_PROJECT_NAME "${CONFIG_SITE_NAME}_SOIL${CONFIG_PLOT_NUMBER}")
    message(STATUS "Building SOIL${CONFIG_PLOT_NUMBER} for site: ${CONFIG_SITE_NAME}")
elseif(DEFINED CONFIG_VALVE AND CONFIG_VALVE)
    set(ACTUAL_PROJECT_NAME "${CONFIG_SITE_NAME}_VALVE${CONFIG_PLOT_NUMBER}")
    message(STATUS "Building VALVE${CONFIG_PLOT_NUMBER} for site: ${CONFIG_SITE_NAME}")
elseif(DEFINED CONFIG_SOLENOID AND CONFIG_SOLENOID)
    set(ACTUAL_PROJECT_NAME "${CONFIG_SITE_NAME}_SOLENOID${CONFIG_PLOT_NUMBER}")
    message(STATUS "Building SOLENOID${CONFIG_PLOT_NUMBER} for site: ${CONFIG_SITE_NAME}")
elseif(DEFINED CONFIG_PUMP AND CONFIG_PUMP)
    set(ACTUAL_PROJECT_NAME "${CONFIG_SITE_NAME}_PUMP")
    message(STATUS "Building PUMP for site: ${CONFIG_SITE_NAME}")
else()
    set(ACTUAL_PROJECT_NAME "soil_moisture")
    message(STATUS "Building default configuration")
endif()

# Debug output
message(STATUS "=== Configuration Debug ===")
message(STATUS "ACTUAL_PROJECT_NAME: ${ACTUAL_PROJECT_NAME}")
message(STATUS "===========================")

# Custom target to handle all image copying after build
add_custom_target(create_images ALL
    DEPENDS ${CMAKE_PROJECT_NAME}.elf
    COMMENT "Creating images with PCB-specific naming"
)

# Copy main binary with appropriate name - FORCE OVERWRITE
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_SOURCE_DIR}/images/${OUTPUT_BINARY_NAME}.bin
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin 
        ${CMAKE_SOURCE_DIR}/images/${ACTUAL_PROJECT_NAME}.bin
        COMMENT "Copying ${ACTUAL_PROJECT_NAME}.bin to images/ (forced overwrite)"
)

# Copy partition table with size-specific naming (determined at build time)
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin 
        ${CMAKE_SOURCE_DIR}/images/$<IF:$<BOOL:${CONFIG_MASTER}>,partition_table_8MB.bin,partition_table_4MB.bin>
    COMMENT "Copying partition table to images/"
    VERBATIM
)

# Copy bootloader with site name
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
        ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin 
        ${CMAKE_SOURCE_DIR}/images/bootloader.bin
    COMMENT "Copying bootloader.bin to images/"
    VERBATIM
)

# Optional: Clean target for images - removes only old files of same type
add_custom_target(clean_images
    COMMAND ${CMAKE_COMMAND} -E remove -f ${CMAKE_SOURCE_DIR}/images/${ACTUAL_PROJECT_NAME}.bin
    COMMENT "Cleaning specific image files for current configuration"
)

# Print summary after build
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E echo ""
    COMMAND ${CMAKE_COMMAND} -E echo "======================================"
    COMMAND ${CMAKE_COMMAND} -E echo "Output Binary: images/${ACTUAL_PROJECT_NAME}.bin"
    COMMAND ${CMAKE_COMMAND} -E echo "======================================"
    COMMAND ${CMAKE_COMMAND} -E echo ""
    VERBATIM
)
