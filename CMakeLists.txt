cmake_minimum_required(VERSION 3.24.3)

# add_definitions(__MISC_VISIBLE)
# set_property(GLOBAL PROPERTY CSTD gnu99)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime-v1)

# if you have an include directory, you can include it here
target_include_directories(app PRIVATE 
                           include/
                           dsdl/uavcan/include
                           libcanard/
                           maritime_ec/include/
)

FILE(GLOB app_sources
    src/*.c 
    maritime_ec/src/*.c
    dsdl/uavcan/src/*.c
    libcanard/canard.c
)

target_sources(app PRIVATE ${app_sources})
