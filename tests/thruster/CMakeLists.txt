cmake_minimum_required(VERSION 3.20.5)

set(MARITIME_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../..)
# For avbotz,pwm-thrusters.yaml
list(APPEND DTS_ROOT ${MARITIME_DIR})

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(thruster_test)

target_include_directories(app PRIVATE ${MARITIME_DIR}/include)
target_sources(app PRIVATE src/main.c ${MARITIME_DIR}/src/thruster.c)
