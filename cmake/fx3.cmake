#
# (C)opyright 2015, Alexander Drozdov
# MIT license
# https://github.com/h4tr3d/fx3-cmake
#
# To use this toolchain to develop for Cypress FX3 simple include it to CMakeLists.txt like
#   include(${CMAKE_SOURCE_DIR}/fx3.cmake)
# BEFORE project declaration (project())
#
# Complete sample for including:
#   cmake_minimum_required(VERSION 2.8.8)
#   set(FX3_SDK_DIR ${CMAKE_SOURCE_DIR}/sdk/1.3.3)
#   include(${CMAKE_SOURCE_DIR}/cmake/fx3.cmake)
#   project(Fx3Project)
#   ...
#
# After that you can use next new commands:
#   fx3_enable_cxx()         - enable C++ support for project. Disabled by default.
#                              You can check it via FX3_CXX variable.
#   fx3_disable_stdc_libs()  - disable linking with standard C and gcc libs. Enabled by default.
#                              Note, that you must provide some function implementations for correct
#                              linking.
#                              You can check it via FX3_USE_STDC_LIB variable.
#   fx3_enable_stdcxx_libs() - enable linking with standard C++ library. Disabled by default (increase
#                              target size). Note, you must provide some implementation for needed
#                              function with disabled std c++ libs and enabled C++ support
#                              You can check it via FX3_USE_STDCXX_LIB variable.
#   fx3_add_target(target ARGS) -
#                              add new target. Use it in same way to `add_executable()`. No known
#                              restrictions for me. This command configure new target to use SDK libs.
#                              Also, generator for img file is configured.
#
# Next files except to be present in CMAKE_SOURCE_DIRECTORY:
# - In all cases:
#   - cyfx_gcc_startup.S    - startup assembler, can be overriden via FX3_CORE_ASM_SRC varible before
#                             fx3.cmake including
#   - make/fx3cpp.ld        - linker script
# - Project with disabled C++:
#   -cyfxtx.c               - can be overriden via FX3_CORE_C_SRC variable
# - Project with enabled C++:
#   - cyfxtx.cpp            - can be overriden via FX3_CORE_CXX_SRC variable
#   - cyfxcppsyscall.cpp    - can be overriden via FX3_CORE_CXX_SRC variable
#
# All files above can be found at the SDK installation directory: FX3_INSTALL_PATH/firmware/common and
# should be distributed with your project.
#
# This script ask some enveropment variables to locate binaries. All of them must be configured during
# normal Cypress SDK installation:
#   ARMGCC_INSTALL_PATH - to locate compilator installation
#   ARMGCC_VERSION      - to set conrete compilator version
#   FX3_INSTALL_PATH    - to locate Cypress SDK path
#
# SDK includes and libraries can be overriden by next variable, declared before fx3.cmake include:
#   FX3_SDK_DIR
# for example:
#   set(FX3_SDK_DIR ${CMAKE_SOURCE_DIR}/sdk/1.3.3)
#   include(${CMAKE_SOURCE_DIR}/fx3.cmake)
#
# Note, that `elf2img` tool searches at the PATH location only.
#
# You can use you own `make` command on Windows. In this case you can pass option -DUSE_SDK_MAKE=Off
# to `cmake` and make command must be present in PATH or pointed via -DCMAKE_MAKE_PROGRAM=...
#
# On Windows you must point generator "MinGW Makefiles" if SDK cs-make is used. Or other valid
# generator if you own make is used.
#

set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_SYSTEM_VERSION 1)
set(CMAKE_SYSTEM_PROCESSOR ARM926EJS)
set(CMAKE_CROSSCOMPILING On)

#
# Options
#
set(USE_SDK_MAKE On CACHE BOOL "Force set MAKE command to the cs-make on Windows")

message(STATUS "ENV ARMGCC_INSTALL_PATH: $ENV{ARMGCC_INSTALL_PATH}")
file(TO_CMAKE_PATH $ENV{ARMGCC_INSTALL_PATH} ARMGCC_INSTALL_PATH)
message(STATUS "ARMGCC_INSTALL_PATH: ${ARMGCC_INSTALL_PATH}")

file(TO_CMAKE_PATH $ENV{FX3_INSTALL_PATH} FX3_INSTALL_PATH)
message(STATUS "FX3_INSTALL_PATH: ${FX3_INSTALL_PATH}")

if (WIN32)
  set(TOOL_EXE ".exe")
endif()

set(CMAKE_C_COMPILER   "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-gcc${TOOL_EXE}")
set(CMAKE_CXX_COMPILER "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-g++${TOOL_EXE}")
set(CMAKE_ASM_COMPILER "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-gcc${TOOL_EXE}")
set(CMAKE_OBJCOPY      "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-objcopy${TOOL_EXE}"
  CACHE FILEPATH "The toolchain objcopy command " FORCE)
if (WIN32 AND USE_SDK_MAKE)
    set(CMAKE_MAKE_PROGRAM "${ARMGCC_INSTALL_PATH}/bin/cs-make${TOOL_EXE}"
        CACHE FILEPATH "The toolchain make command" FORCE)
endif()
# If sets, AR "linking" will be fails. See: http://www.cmake.org/pipermail/cmake/2010-March/035540.html
#set(CMAKE_AR           "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-ar"
#  CACHE FILEPATH "The toolchain ar command " FORCE)
set(CMAKE_LINKER       "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-ld${TOOL_EXE}"
  CACHE FILEPATH "The toolchain linker command " FORCE)
set(CMAKE_EXE_SIZE     "${ARMGCC_INSTALL_PATH}/bin/arm-none-eabi-size${TOOL_EXE}")

if (WIN32)
    set(CMAKE_EXE_ELF2IMG  "${FX3_INSTALL_PATH}/util/elf2img/elf2img.exe")
else()
    set(CMAKE_EXE_ELF2IMG  elf2img)
endif()

# FX3 Targets output
set(BINARY_POSTFIX ".elf")

set(CMAKE_SHARED_LIBRARY_LINK_C_FLAGS "")
set(CMAKE_SHARED_LIBRARY_LINK_CXX_FLAGS "")
# WA for cmake bug
#set(CMAKE_C_COMPILER_WORKS 1)
#set(CMAKE_CXX_COMPILER_WORKS 1)

# adjust the default behaviour of the FIND_XXX() commands:
# search headers and libraries in the target environment, search 
# programs in the host environment
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY BOTH)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE BOTH)

enable_language(C)
enable_language(ASM)
enable_language(CXX)

set(FX3FWROOT ${FX3_INSTALL_PATH}/firmware)
set(FX3PFWROOT ${FX3FWROOT}/u3p_firmware)

if (${FX3_SDK_DIR})
    set(FX3PFWROOT ${FX3_SDK_DIR}/u3p_firmware)
endif()

include_directories(${FX3PFWROOT}/inc)
include_directories(${CMAKE_SOURCE_DIR})

unset(CMAKE_C_FLAGS)
unset(CMAKE_C_FLAGS_DEBUG)
unset(CMAKE_C_FLAGS_RELEASE)
unset(CMAKE_CXX_FLAGS)
unset(CMAKE_CXX_FLAGS_DEBUG)
unset(CMAKE_CXX_FLAGS_RELEASE)
unset(CMAKE_ASM_FLAGS)
unset(CMAKE_ASM_FLAGS_DEBUG)
unset(CMAKE_ASM_FLAGS_RELEASE)

set(CMAKE_ASM_FLAGS    "-Wall -c -mcpu=arm926ej-s -mthumb-interwork -Wno-write-strings")
set(CMAKE_FLAGS_COMMON "-DTX_ENABLE_EVENT_TRACE -DCYU3P_FX3=1 -D__CYU3P_TX__=1 -Wall -mcpu=arm926ej-s -mthumb-interwork -Wno-write-strings")
set(CMAKE_FLAGS_COMMON_DEBUG "-O2 -DDEBUG -g")
set(CMAKE_FLAGS_COMMON_RELEASE "-Os")
set(CMAKE_C_FLAGS         ${CMAKE_FLAGS_COMMON})
set(CMAKE_C_FLAGS_DEBUG   ${CMAKE_FLAGS_COMMON_DEBUG})
set(CMAKE_C_FLAGS_RELEASE ${CMAKE_FLAGS_COMMON_RELEASE})
set(CMAKE_CXX_FLAGS         ${CMAKE_FLAGS_COMMON})
set(CMAKE_CXX_FLAGS_DEBUG   ${CMAKE_FLAGS_COMMON_DEBUG})
set(CMAKE_CXX_FLAGS_RELEASE ${CMAKE_FLAGS_COMMON_RELEASE})

# -Map $(MODULE).map
set(CMAKE_EXE_LINKER_FLAGS
  "-T \"${CMAKE_SOURCE_DIR}/make/fx3cpp.ld\" -d --gc-sections --no-wchar-size-warning -nostdlib -nostartfiles --entry CyU3PFirmwareEntry")

# Set default build type
if (NOT CMAKE_BUILD_TYPE)
  message(STATUS "Set default build type to Release")
  set(CMAKE_BUILD_TYPE Release
    CACHE STRING "Choose the type of build, options are: Debug Release RelWithDebInfo MinSizeRel." FORCE)
endif()

# Per build type settings
if (CMAKE_BUILD_TYPE MATCHES Debug)
  set(CYCONFOPT "fx3_debug")
else()
  set(CYCONFOPT "fx3_release")
endif()

# FX3 SDK C++ Usage. Use `fx3_enable_cxx()` call to enable C++ in project.
set(FX3_CXX OFF)

# Library usage information
set(FX3_USE_STDC_LIB   ON)
set(FX3_USE_STDCXX_LIB OFF)

# Base Cypress SDK libs
set(FX3_LIBS
  "${FX3PFWROOT}/lib/${CYCONFOPT}/cyu3sport.a"
  "${FX3PFWROOT}/lib/${CYCONFOPT}/cyu3lpp.a"
  "${FX3PFWROOT}/lib/${CYCONFOPT}/cyfxapi.a"
  "${FX3PFWROOT}/lib/${CYCONFOPT}/cyu3threadx.a"
  )

# Standard C library
set(FX3_C_LIBS
  "${ARMGCC_INSTALL_PATH}/arm-none-eabi/lib/libm.a"
  "${ARMGCC_INSTALL_PATH}/arm-none-eabi/lib/libc.a"
  "${ARMGCC_INSTALL_PATH}/lib/gcc/arm-none-eabi/$ENV{ARMGCC_VERSION}/libgcc.a"
  )

# STD C++ Library
set(FX3_CXX_LIBS
  "${ARMGCC_INSTALL_PATH}/arm-none-eabi/lib/libstdc++.a"
  )

# Core sources
if ("${FX3_CORE_ASM_SRC}" STREQUAL "")
    set(FX3_CORE_ASM_SRC
        ${CMAKE_SOURCE_DIR}/cyfx_gcc_startup.S
    )
endif()

if ("${FX3_CORE_C_SRC}" STREQUAL "")
    set(FX3_CORE_C_SRC
        ${CMAKE_SOURCE_DIR}/cyfxtx.c
    )
endif()

if ("${FX3_CORE_CXX_SRC}" STREQUAL "")
    set(FX3_CORE_CXX_SRC
        ${CMAKE_SOURCE_DIR}/cyfxtx.cpp
        ${CMAKE_SOURCE_DIR}/cyfxcppsyscall.cpp
    )
endif()

# Linker patterns
set(CMAKE_C_LINK_EXECUTABLE   "<CMAKE_LINKER> <OBJECTS> <LINK_LIBRARIES> <CMAKE_C_LINK_FLAGS> <LINK_FLAGS> -Map <TARGET_BASE>.map -o <TARGET>")
set(CMAKE_CXX_LINK_EXECUTABLE "<CMAKE_LINKER> <OBJECTS> <LINK_LIBRARIES> <CMAKE_CXX_LINK_FLAGS> <LINK_FLAGS> -Map <TARGET_BASE>.map -o <TARGET>")

function(add_img_generator target)
    if (${CMAKE_VERSION} VERSION_GREATER "3.0.0")
        set(location $<TARGET_FILE:${target}>)
    else()
        get_property(location TARGET ${target} PROPERTY LOCATION)
    endif()

    get_property(name TARGET ${target} PROPERTY OUTPUT_NAME)
    if (NOT name)
        get_property(name TARGET ${target} PROPERTY NAME)
    endif()

    if (${name} MATCHES "${BINARY_POSTFIX}$")
        string(REGEX REPLACE "${BINARY_POSTFIX}$" "" name ${name})
    endif()

    message(STATUS "name: ${name}, location: ${location}")

    add_custom_command(
        TARGET ${target} POST_BUILD
        COMMAND ${CMAKE_EXE_ELF2IMG} -i ${location} -o ${name}.img
        WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
        COMMENT "Convert the ELF output file to a binary image")
endfunction()

function(add_size_statistics target)
    if (${CMAKE_VERSION} VERSION_GREATER "3.0.0")
        set(location $<TARGET_FILE:${target}>)
    else()
        get_property(location TARGET ${target} PROPERTY LOCATION)
    endif()
    add_custom_command(
      TARGET ${target} POST_BUILD
      COMMAND ${CMAKE_EXE_SIZE} ${location}
      WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
      COMMENT "Firmware size:")
endfunction()

# Helper function
function(fx3_get_core_src var)
  set(std_src_list ${FX3_CORE_ASM_SRC})

  if(FX3_CXX)
    list(APPEND std_src_list ${FX3_CORE_CXX_SRC})
  else()
    list(APPEND std_src_list ${FX3_CORE_C_SRC})
  endif()

  set(var ${std_src_list} PARENT_SCOPE)
endfunction()

# Add FX3 target
function(fx3_add_target target)

  set(std_src_list ${FX3_CORE_ASM_SRC})
  if(FX3_CXX)
    list(APPEND std_src_list ${FX3_CORE_CXX_SRC})
  else()
    list(APPEND std_src_list ${FX3_CORE_C_SRC})
  endif()

  add_executable(${target} ${ARGN} ${std_src_list})
  # Add ELF postfix
  get_property(name TARGET ${target} PROPERTY OUTPUT_NAME)
  if (NOT name OR NOT ${name} MATCHES "${BINARY_POSTFIX}$")
    set_target_properties(${target} PROPERTIES OUTPUT_NAME "${target}${BINARY_POSTFIX}")
  endif()

  add_size_statistics(${target})
  add_img_generator(${target})

  set_target_properties(${target} PROPERTIES ENABLE_EXPORTS OFF)

  # Base SDK libs
  #target_link_libraries(${target} ${target}_core ${FX3_LIBS})
  target_link_libraries(${target} ${FX3_LIBS})

  if (FX3_USE_STDC_LIB)
    target_link_libraries(${target} ${FX3_C_LIBS})

    # STDCXX without STDC impossible
    if (FX3_USE_STDCXX_LIB)
      target_link_libraries(${target} ${FX3_CXX_LIBS})
    endif()
  endif()

  #target_link_libraries(${target} ${target}_core)
endfunction()

# Add FX3 libraries to link
function(fx3_target_link_libraries target)
  # Base SDK libs
  target_link_libraries(${target} ${target}_core ${FX3_LIBS})
  #target_link_libraries(${target} ${FX3_LIBS})

  if (FX3_USE_STDC_LIB)
    target_link_libraries(${target} ${FX3_C_LIBS})

    # STDCXX without STDC impossible
    if (FX3_USE_STDCXX_LIB)
      target_link_libraries(${target} ${FX3_CXX_LIBS})
    endif()
  endif()

  target_link_libraries(${target} ${target}_core)
endfunction()

# Enable C++ support for project
function(fx3_enable_cxx)
  set(FX3_CXX ON PARENT_SCOPE)
endfunction()

# Disable STD C libs
function(fx3_disable_stdc_libs)
  set(FX3_USE_STDC_LIB OFF PARENT_SCOPE)
endfunction()

# Enable STD C++ libs
function(fx3_enable_stdcxx_libs)
  set(FX3_USE_STDC_LIB  ON PARENT_SCOPE)
  set(FX3_USE_STDCXX_LIB ON PARENT_SCOPE)
endfunction()
