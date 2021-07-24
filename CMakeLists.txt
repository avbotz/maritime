cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
	src/thruster/csr.c
	src/thruster/thruster.c
	src/thruster/dummy_thruster_output.c
	src/main.c
)
