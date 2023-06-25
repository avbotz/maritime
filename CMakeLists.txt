cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

FILE(GLOB app_sources 
	src/pressure.c
	src/ads1115.c
	#src/*.c 
	extern/maritime_ec/src/*.c
)
target_sources(app PRIVATE ${app_sources})

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/extern/maritime_ec/include
)
