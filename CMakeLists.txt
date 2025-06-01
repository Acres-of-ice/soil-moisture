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

# Determine project name based on configuration
if(CONFIG_MASTER)
    set(PROJECT_NAME_CUSTOM "${CONFIG_SITE_NAME}_MASTER")
elseif(CONFIG_SOIL)
    set(PROJECT_NAME_CUSTOM "${CONFIG_SITE_NAME}_SOIL${CONFIG_PLOT_NUMBER}")
elseif(CONFIG_VALVE)
    set(PROJECT_NAME_CUSTOM "${CONFIG_SITE_NAME}_VALVE${CONFIG_PLOT_NUMBER}")
elseif(CONFIG_SOLENOID)
    set(PROJECT_NAME_CUSTOM "${CONFIG_SITE_NAME}_SOLENOID${CONFIG_PLOT_NUMBER}")
elseif(CONFIG_PUMP)
    set(PROJECT_NAME_CUSTOM "${CONFIG_SITE_NAME}_PUMP")
else()
    set(PROJECT_NAME_CUSTOM "soil_moisture")
endif()

project(${PROJECT_NAME_CUSTOM} VERSION ${BASE_VERSION})

# Enhanced build configuration display function
function(display_build_config)
    # Determine chip type from IDF_TARGET
    set(CHIP_TYPE "Unknown")
    if(DEFINED IDF_TARGET)
        string(TOUPPER ${IDF_TARGET} CHIP_TYPE)
    elseif(DEFINED CONFIG_IDF_TARGET)
        string(TOUPPER ${CONFIG_IDF_TARGET} CHIP_TYPE)
    endif()
    
    # Calculate device address and determine device info
    set(DEVICE_INFO "")
    set(DEVICE_ADDR "")
    
    if(CONFIG_MASTER)
        set(DEVICE_INFO "MASTER")
        set(DEVICE_ADDR "0x01")
    elseif(CONFIG_SOIL)
        math(EXPR SOIL_ADDR "32 + ${CONFIG_PLOT_NUMBER}")  # 0x20 + plot
        string(TOUPPER "${SOIL_ADDR}" SOIL_ADDR_HEX)
        math(EXPR SOIL_ADDR_HEX "${SOIL_ADDR}" OUTPUT_FORMAT HEXADECIMAL)
        set(DEVICE_INFO "SOIL ${CONFIG_PLOT_NUMBER}")
        set(DEVICE_ADDR "${SOIL_ADDR_HEX}")
    elseif(CONFIG_VALVE)
        math(EXPR VALVE_ADDR "16 + ${CONFIG_PLOT_NUMBER}")  # 0x10 + plot
        string(TOUPPER "${VALVE_ADDR}" VALVE_ADDR_HEX)
        math(EXPR VALVE_ADDR_HEX "${VALVE_ADDR}" OUTPUT_FORMAT HEXADECIMAL)
        set(DEVICE_INFO "VALVE ${CONFIG_PLOT_NUMBER}")
        set(DEVICE_ADDR "${VALVE_ADDR_HEX}")
    elseif(CONFIG_SOLENOID)
        math(EXPR SOLENOID_ADDR "96 + ${CONFIG_PLOT_NUMBER}")  # 0x60 + plot
        string(TOUPPER "${SOLENOID_ADDR}" SOLENOID_ADDR_HEX)
        math(EXPR SOLENOID_ADDR_HEX "${SOLENOID_ADDR}" OUTPUT_FORMAT HEXADECIMAL)
        set(DEVICE_INFO "SOLENOID ${CONFIG_PLOT_NUMBER}")
        set(DEVICE_ADDR "${SOLENOID_ADDR_HEX}")
    elseif(CONFIG_PUMP)
        set(DEVICE_INFO "PUMP")
        set(DEVICE_ADDR "0x51")
    endif()
    
    # Hardware features detection
    set(HW_FEATURES "")
    if(CONFIG_ENABLE_GSM)
        list(APPEND HW_FEATURES "GSM")
    endif()
    if(CONFIG_ENABLE_VALVE)
        list(APPEND HW_FEATURES "Valves")
    endif()
    if(CONFIG_ENABLE_TEMP_HUMIDITY)
        list(APPEND HW_FEATURES "TempHum")
    endif()
    if(CONFIG_ENABLE_FLOWMETER)
        list(APPEND HW_FEATURES "Flow")
    endif()
    if(CONFIG_ENABLE_PRESSURE)
        list(APPEND HW_FEATURES "Pressure")
    endif()
    if(CONFIG_ENABLE_VOLTAGE_CUTOFF)
        list(APPEND HW_FEATURES "VoltCutoff")
    endif()
    if(CONFIG_ENABLE_SIMULATION_MODE)
        list(APPEND HW_FEATURES "SIMULATION")
    endif()
    
    # Join features with commas
    string(JOIN ", " HW_FEATURES_STR ${HW_FEATURES})
    if(NOT HW_FEATURES_STR)
        set(HW_FEATURES_STR "None")
    endif()
    
    # Color codes for prominent display
    string(ASCII 27 ESC)
    set(BOLD "${ESC}[1m")
    set(GREEN "${ESC}[32m")
    set(BLUE "${ESC}[34m")
    set(CYAN "${ESC}[36m")
    set(YELLOW "${ESC}[33m")
    set(RED "${ESC}[31m")
    set(RESET "${ESC}[0m")
    
    # Display the configuration with enhanced formatting
    message(STATUS "")
    message(STATUS "${BOLD}${GREEN}╔══════════════════════════════════════════════════════════════════════════════╗${RESET}")
    message(STATUS "${BOLD}${GREEN}║                              BUILD CONFIGURATION                            ║${RESET}")
    message(STATUS "${BOLD}${GREEN}╠══════════════════════════════════════════════════════════════════════════════╣${RESET}")
    message(STATUS "${GREEN}║${RESET}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Version:${RESET}      ${BOLD}${YELLOW}${PROJECT_VERSION}${RESET}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Site:${RESET}         ${BOLD}${CONFIG_SITE_NAME}${RESET}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Device:${RESET}       ${BOLD}${BLUE}${DEVICE_INFO}${RESET} ${CYAN}(Address: ${DEVICE_ADDR})${RESET}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Chip:${RESET}         ${CHIP_TYPE}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Plots:${RESET}        ${CONFIG_NUM_PLOTS}")
    message(STATUS "${GREEN}║${RESET} ${BOLD}${CYAN}Features:${RESET}     ${HW_FEATURES_STR}")
    
    # Special warnings for important configurations
    if(CONFIG_ENABLE_SIMULATION_MODE)
        message(STATUS "${GREEN}║${RESET}")
        message(STATUS "${GREEN}║${RESET} ${BOLD}${RED}⚠ WARNING: SIMULATION MODE ENABLED ⚠${RESET}")
    endif()
    
    if(CONFIG_SITE_TEST)
        message(STATUS "${GREEN}║${RESET}")
        message(STATUS "${GREEN}║${RESET} ${BOLD}${YELLOW}ℹ INFO: TEST SITE CONFIGURATION${RESET}")
    endif()
    
    message(STATUS "${GREEN}║${RESET}")
    message(STATUS "${BOLD}${GREEN}╚══════════════════════════════════════════════════════════════════════════════╝${RESET}")
    message(STATUS "")
    
    # Get user confirmation
    message(STATUS "${BOLD}${CYAN}Please verify the above configuration is correct.${RESET}")
    message(STATUS "${CYAN}Press ENTER to continue or Ctrl+C to abort...${RESET}")
    
    # Check if we're in a CI environment or automated build
    if(DEFINED ENV{CI} OR DEFINED ENV{GITHUB_ACTIONS} OR DEFINED ENV{GITLAB_CI} OR DEFINED ENV{BUILD_AUTOMATION})
        message(STATUS "${YELLOW}Automated build environment detected - skipping user confirmation${RESET}")
    else()
        # Pause for user input
        execute_process(
            COMMAND ${CMAKE_COMMAND} -E echo ""
            COMMAND bash -c "read -p ''"
            WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
            ERROR_QUIET
        )
    endif()
    
    message(STATUS "${GREEN}✓ Configuration confirmed - proceeding with build...${RESET}")
    message(STATUS "")
endfunction()

# Call the enhanced display function early in the build process
display_build_config()

# Optional: Print version information
# message(STATUS "Flashing webapp")
# spiffs_create_partition_image(storage spiffs_image FLASH_IN_PROJECT)

# # Create images directory
# file(MAKE_DIRECTORY ${CMAKE_SOURCE_DIR}/images)
#
# # Determine output filename based on configuration
# set(OUTPUT_BINARY_NAME ${CMAKE_PROJECT_NAME})
#
# if(CONFIG_MASTER AND CONFIG_SITE_NAME)
#     set(OUTPUT_BINARY_NAME "${CONFIG_SITE_NAME}_MASTER")
# elseif(CONFIG_SOIL_A)
#     set(OUTPUT_BINARY_NAME "SOIL_A")
# elseif(CONFIG_SOIL_B) 
#     set(OUTPUT_BINARY_NAME "SOIL_B")
# elseif(CONFIG_VALVE_A)
#     set(OUTPUT_BINARY_NAME "VALVE_A")
# elseif(CONFIG_VALVE_B)
#     set(OUTPUT_BINARY_NAME "VALVE_B")
# elseif(CONFIG_PUMP)
#     set(OUTPUT_BINARY_NAME "PUMP")
# endif()
#
# # Custom target to handle all image copying
# add_custom_target(create_images ALL
#     DEPENDS ${CMAKE_PROJECT_NAME}.elf
#     COMMENT "Creating images with proper naming"
# )
#
# # Copy main binary with appropriate name
# add_custom_command(TARGET create_images POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy 
#         ${CMAKE_BINARY_DIR}/${CMAKE_PROJECT_NAME}.bin 
#         ${CMAKE_SOURCE_DIR}/images/${OUTPUT_BINARY_NAME}.bin
#     COMMENT "Copying ${OUTPUT_BINARY_NAME}.bin to images/"
# )
#
# # Copy partition table with size-specific naming
# if(CONFIG_MASTER)
#     set(PARTITION_TABLE_NAME "partition_table_8MB.bin")
# else()
#     set(PARTITION_TABLE_NAME "partition_table_4MB.bin")
# endif()
#
# add_custom_command(TARGET create_images POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy 
#         ${CMAKE_BINARY_DIR}/partition_table/partition-table.bin 
#         ${CMAKE_SOURCE_DIR}/images/${PARTITION_TABLE_NAME}
#     COMMENT "Copying ${PARTITION_TABLE_NAME} to images/"
# )
#
# # Copy bootloader
# add_custom_command(TARGET create_images POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy 
#         ${CMAKE_BINARY_DIR}/bootloader/bootloader.bin 
#         ${CMAKE_SOURCE_DIR}/images/bootloader.bin
#     COMMENT "Copying bootloader.bin to images/"
# )
#
# # Optional: Clean target for images
# add_custom_target(clean_images
#     COMMAND ${CMAKE_COMMAND} -E remove_directory ${CMAKE_SOURCE_DIR}/images
#     COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_SOURCE_DIR}/images
#     COMMENT "Cleaning images directory"
# )
