#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

cmake_minimum_required(VERSION 3.16)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

project(hamran-tools LANGUAGES CXX VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GitFlow)
include(GNUInstallDirs)

## Initialze external git submodule projects

find_package(Git QUIET)
if(GIT_FOUND AND EXISTS "${PROJECT_SOURCE_DIR}/.git")
# Update submodules as needed
    option(GIT_SUBMODULE "Check submodules during build" ON)
    if(GIT_SUBMODULE)
        message(STATUS "Submodule update")
        execute_process(COMMAND ${GIT_EXECUTABLE} submodule update --init --recursive
                        WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                        RESULT_VARIABLE GIT_SUBMOD_RESULT)
        if(NOT GIT_SUBMOD_RESULT EQUAL "0")
            message(FATAL_ERROR "git submodule update --init --recursive failed with ${GIT_SUBMOD_RESULT}, please checkout submodules")
        endif()
    endif()
endif()

# if(NOT EXISTS "${PROJECT_SOURCE_DIR}/extern/repo/CMakeLists.txt")
#     message(FATAL_ERROR "The submodules were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
# endif()

## Set up the external projects

#find_package(limesuiteng 0.3.0 REQUIRED)
set(ENABLE_DOXYGEN OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_DOCS_DOXYGEN OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_SHARED_LIBS OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_CLI OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_DRIVERS_LIMEPCIE OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_EXAMPLES OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_GUI OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_CLI OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_PLUGINS OFF CACHE BOOL "preset for hamran-tools")
set(INSTALL_UDEV_RULES OFF CACHE BOOL "preset for hamran-tools")
set(UDEV_RULES_RELOAD_ON_INSTALL OFF CACHE BOOL "preset for hamran-tools")
set(LIMEPCIE_INSTALL OFF CACHE BOOL "preset for hamran-tools")
set(LIMEPCIE_RELOAD_ON_INSTALL OFF CACHE BOOL "preset for hamran-tools")
add_subdirectory(extern/LimeSuiteNG)

#find_package(Threads)
#find_package(cpptrace REQUIRED)
#find_package(streampu REQUIRED)
set(SPU_TESTS OFF CACHE BOOL "preset for hamran-tools")
set(SPU_COMPILE_STATIC_LIB ON BOOL "preset for hamran-tools")
add_subdirectory(extern/streampu)
add_library(spu::spu-static-lib ALIAS spu-static-lib)

#find_package(CLI11 2.5.0 REQUIRED)
add_subdirectory(extern/CLI11)

## Set up the hamran-tools project

add_subdirectory(apps)

add_custom_target(doc SOURCES
    README.md
)

include(CPack)
