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

project(soil_moisture VERSION ${BASE_VERSION})

# Optional: Print version information
message(STATUS "Full version: ${PROJECT_VERSION}")

# spiffs_create_partition_image(storage spiffs_image FLASH_IN_PROJECT)

# Create images directory
file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/images)

# Determine output filename based on configuration
set(OUTPUT_BINARY_NAME ${CMAKE_PROJECT_NAME})

if(CONFIG_MASTER AND CONFIG_SITE_NAME)
    set(OUTPUT_BINARY_NAME "${CONFIG_SITE_NAME}_MASTER")
elseif(CONFIG_SOIL_A)
    set(OUTPUT_BINARY_NAME "SOIL_A")
elseif(CONFIG_SOIL_B) 
    set(OUTPUT_BINARY_NAME "SOIL_B")
elseif(CONFIG_VALVE_A)
    set(OUTPUT_BINARY_NAME "VALVE_A")
elseif(CONFIG_VALVE_B)
    set(OUTPUT_BINARY_NAME "VALVE_B")
elseif(CONFIG_PUMP)
    set(OUTPUT_BINARY_NAME "PUMP")
endif()

# Custom target to handle all image copying
add_custom_target(create_images ALL
    DEPENDS ${CMAKE_PROJECT_NAME}.elf
    COMMENT "Creating images with proper naming"
)

# Copy main binary with appropriate name
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin 
        ${CMAKE_SOURCE_DIR}/images/${OUTPUT_BINARY_NAME}.bin
    COMMENT "Copying ${OUTPUT_BINARY_NAME}.bin to images/"
)

# Copy partition table with size-specific naming
if(CONFIG_MASTER)
    set(PARTITION_TABLE_NAME "partition_table_8MB.bin")
else()
    set(PARTITION_TABLE_NAME "partition_table_4MB.bin")
endif()

add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin 
        ${CMAKE_SOURCE_DIR}/images/${PARTITION_TABLE_NAME}
    COMMENT "Copying ${PARTITION_TABLE_NAME} to images/"
)

# Copy bootloader
add_custom_command(TARGET create_images POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy 
        ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin 
        ${CMAKE_SOURCE_DIR}/images/bootloader.bin
    COMMENT "Copying bootloader.bin to images/"
)

# Optional: Clean target for images
add_custom_target(clean_images
    COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/images
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/images
    COMMENT "Cleaning images directory"
)
