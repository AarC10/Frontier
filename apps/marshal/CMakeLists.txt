cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(marshal LANGUAGES C)

target_include_directories(app PRIVATE include)
target_sources(app PRIVATE src/main.cpp src/device_runtime.cpp src/sim_pipe_reader.c src/usb_support.cpp)
