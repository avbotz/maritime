cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

target_sources(app PRIVATE
	src/ahrs.c
	src/dvl.c
	src/killswitch.c
	src/main.cpp
	src/pressure.c
	src/servos.c
	src/util.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

add_subdirectory(extern/maritime_ec)
include_directories(include extern/maritime_ec/include)