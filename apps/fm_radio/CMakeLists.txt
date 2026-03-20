cmake_minimum_required(VERSION 3.13.1)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})

project(fm_radio LANGUAGES C)

target_include_directories(app PRIVATE include ../../drivers/rda5807m)
target_sources(app PRIVATE src/main.cpp)
