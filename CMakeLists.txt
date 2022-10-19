cmake_minimum_required(VERSION 3.20.0)

set(BOARD rpi_pico)

find_package(Zephyr)
project(lack_brake_lights)

target_sources(app PRIVATE src/main.c)
