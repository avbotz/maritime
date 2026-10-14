cmake_minimum_required(VERSION 3.20.5)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

target_include_directories(app PRIVATE include)

FILE(GLOB app_sources src/*.c)

target_sources(app PRIVATE ${app_sources})
