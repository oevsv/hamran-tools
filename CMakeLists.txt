#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

cmake_minimum_required(VERSION 3.16)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

project(hamran-tools LANGUAGES C CXX VERSION 0.0.2)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GitFlow)
include(GNUInstallDirs)

include(GitSubmodule)

## Set up the external projects

##
## LimesuiteNG
##

message(STATUS "Configure LimeSuiteNG")
set(ENABLE_DOXYGEN               OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_DOCS_DOXYGEN           OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_SHARED_LIBS            OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_CLI                    OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_DRIVERS_LIMEPCIE       OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_EXAMPLES               OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_GUI                    OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_CLI                    OFF CACHE BOOL "preset for hamran-tools")
set(BUILD_PLUGINS                OFF CACHE BOOL "preset for hamran-tools")
set(INSTALL_UDEV_RULES           OFF CACHE BOOL "preset for hamran-tools")
set(UDEV_RULES_RELOAD_ON_INSTALL OFF CACHE BOOL "preset for hamran-tools")
set(LIMEPCIE_INSTALL             OFF CACHE BOOL "preset for hamran-tools")
set(LIMEPCIE_RELOAD_ON_INSTALL   OFF CACHE BOOL "preset for hamran-tools")
set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)
add_subdirectory(extern/LimeSuiteNG EXCLUDE_FROM_ALL)

##
## liquid-dsp
##

message(STATUS "Configure liquid-dsp")
set(liquid_SHARED_LIBS          OFF)
add_subdirectory(extern/liquid-dsp EXCLUDE_FROM_ALL)

##
## CLI11
##

message(STATUS "Configure CLI11")
add_subdirectory(extern/CLI11 EXCLUDE_FROM_ALL)

##
## PlatformFolders
##

message(STATUS "Configure PlatformFolders")
add_subdirectory(extern/PlatformFolders EXCLUDE_FROM_ALL)

##
## spdlog
##

message(STATUS "Configure spdlog")
set(SPDLOG_USE_STD_FORMAT       ON CACHE BOOL "preset for hamran-tools")
add_subdirectory(extern/spdlog EXCLUDE_FROM_ALL)

##
## cpp-terminal
##

message(STATUS "Configure cpp-terminal")
add_subdirectory(extern/cpp-terminal EXCLUDE_FROM_ALL)


##
## Set up the hamran-tools project
##

message(STATUS "Configure hamran-tools")
add_subdirectory(apps)

add_custom_target(doc SOURCES
    README.md
)

include(CPack)
