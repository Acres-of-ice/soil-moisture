cmake_minimum_required(VERSION 3.16)

set(EXTRA_COMPONENT_DIRS ${CMAKE_SOURCE_DIR}/components)
set(SPIFFS_IMAGE_SOURCE_DIR "${CMAKE_SOURCE_DIR}/spiffs")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)


project(soil_moisture)
spiffs_create_partition_image(spiffs spiffs_image FLASH_IN_PROJECT)


