cmake_minimum_required(VERSION 3.5)

# Add component directories
set(EXTRA_COMPONENT_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-libopus
    ${CMAKE_CURRENT_SOURCE_DIR}/components/esp-sr
)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(aec_test_diff_buffer)