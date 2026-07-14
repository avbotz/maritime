cmake_minimum_required(VERSION 3.20.5)
find_package(Zephyr)
project(maritime)

# if you have an include directory, you can include it here
target_include_directories(app PRIVATE include)

# FILE(GLOB app_sources src/*.c)

# target_sources(app PRIVATE ${app_sources})
target_sources(app PRIVATE src/main.c src/killswitch.c src/thruster.c src/util.c src/pressure.c src/servo.c src/ahrs.c)
