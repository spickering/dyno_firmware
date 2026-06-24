#
# Low-power BLE NUS demo for Seeed XIAO nRF52840 Sense
# nRF Connect SDK v3.3.0
#
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(xiao_lp_ble)

target_sources(app PRIVATE
	src/main.c
	src/ads1220.c
)

target_include_directories(app PRIVATE src)
