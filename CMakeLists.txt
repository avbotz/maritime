cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(maritime)

target_include_directories(app PRIVATE include)

target_sources(app PRIVATE
	src/pubsub/pubsub.c
	#src/t1.c
	#src/t2.c
	#src/t3.c
	src/thruster/csr.c
	src/thruster/thruster.c
	src/main.c
)
