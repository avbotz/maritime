cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

target_sources(app PRIVATE
	src/ahrs.c
	src/dvl.c
	src/killswitch.c
	src/main.c
	src/pressure.c
	src/servos.c
	src/util.c
)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

#file(GLOB MEC_SOURCES extern/maritime_ec/src/*.cpp)
#add_library(maritime_ec
 #   ${MEC_SOURCES}
#)
#target_include_directories(maritime_ec PUBLIC
#    ${CMAKE_CURRENT_SOURCE_DIR}/extern/maritime_ec/include
#)
#target_link_libraries(app PRIVATE maritime_ec)

add_subdirectory(extern/maritime_ec)
target_link_libraries(app PRIVATE maritime_ec)
#include_directories(extern/maritime_ec/include)
