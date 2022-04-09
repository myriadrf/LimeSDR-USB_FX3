cmake_minimum_required(VERSION 2.8.8)
include(${CMAKE_SOURCE_DIR}/cmake/fx3.cmake)
project(Fx3CmakeSample)

set(WPEDANTIC OFF CACHE BOOL "turn on/off -Wpedantic option for compiller (default: off)")
set(WERROR    OFF CACHE BOOL "turn on/off -Werror option for compiller (default: off)")

# Project settings
#fx3_disable_stdc_libs()

# Additional C++ flags
set(COMMON_FLAGS "-fmessage-length=0 -ffunction-sections -fdata-sections -fshort-wchar -Wall -Wextra")
if (WPEDANTIC)
  set(COMMON_FLAGS "${COMMON_FLAGS} -Wpedantic")
endif()
if (WERROR)
  set(COMMON_FLAGS "${COMMON_FLAGS} -Werror")
endif()
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${COMMON_FLAGS} -Wwrite-strings -std=c99")

# source files
set(FW_SRC 
    main.c
    spi_flash_lib.c
    usb_descriptors.c
)
fx3_add_target(LimeSDR-USB_fx3_fw ${FW_SRC})
