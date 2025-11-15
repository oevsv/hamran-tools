#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

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

## Give a nice error message
file(GLOB EXTERN LIST_DIRECTORIES true  "${CMAKE_CURRENT_LIST_DIR}/*")
foreach(E ${EXTERN})
    if(IS_DIRECTORY ${E} AND NOT EXISTS "${E}/.git")
        message(FATAL_ERROR "The submodules in ${E} were not downloaded! GIT_SUBMODULE was turned off or failed. Please update submodules and try again.")
    endif()
endforeach()

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
add_subdirectory(LimeSuiteNG EXCLUDE_FROM_ALL)

##
## liquid-dsp
##

message(STATUS "Configure liquid-dsp")
set(liquid_SHARED_LIBS          OFF)
add_subdirectory(liquid-dsp EXCLUDE_FROM_ALL)

##
## CLI11
##

message(STATUS "Configure CLI11")
add_subdirectory(CLI11 EXCLUDE_FROM_ALL)

##
## PlatformFolders
##

message(STATUS "Configure PlatformFolders")
add_subdirectory(PlatformFolders EXCLUDE_FROM_ALL)

##
## spdlog
##

message(STATUS "Configure spdlog")
set(SPDLOG_USE_STD_FORMAT       ON CACHE BOOL "preset for hamran-tools")
add_subdirectory(spdlog EXCLUDE_FROM_ALL)

##
## cpp-terminal
##

message(STATUS "Configure cpp-terminal")
add_subdirectory(cpp-terminal EXCLUDE_FROM_ALL)

##
## libtuntap
##

message(STATUS "Configure libtuntap")
add_subdirectory(libtuntap EXCLUDE_FROM_ALL)

