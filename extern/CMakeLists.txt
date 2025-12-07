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
set(ENABLE_DOXYGEN               OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_DOCS_DOXYGEN           OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_SHARED_LIBS            OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_CLI                    OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_DRIVERS_LIMEPCIE       OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_EXAMPLES               OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_GUI                    OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_CLI                     ON CACHE BOOL "preset for hamran-tools" FORCE)
set(BUILD_PLUGINS                OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(INSTALL_UDEV_RULES           OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(UDEV_RULES_RELOAD_ON_INSTALL OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(LIMEPCIE_INSTALL             OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(LIMEPCIE_RELOAD_ON_INSTALL   OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(CMAKE_POLICY_DEFAULT_CMP0135 NEW)
add_subdirectory(LimeSuiteNG EXCLUDE_FROM_ALL)

# selectively install  command line tools from limeSuiteNG
add_custom_target(LimeSuiteNG_selective_all ALL)
add_dependencies(LimeSuiteNG_selective_all limeFLASH)

# rename the tool to avoid clash with upstream original
set_target_properties(limeFLASH PROPERTIES OUTPUT_NAME lm2-FLASH)
install(TARGETS
    limeFLASH
)

##
## liquid-dsp
##

message(STATUS "Configure liquid-dsp")

#TODO: This is a hack, the upstream liquid-dsp should be fixed instead.
set(liquid_SHARED_LIBS          OFF CACHE BOOL "preset for hamran-tools" FORCE)
set(FIND_FFTW                   OFF CACHE BOOL "preset for hamran-tools" FORCE)

add_subdirectory(liquid-dsp EXCLUDE_FROM_ALL)
target_compile_options(fec PRIVATE "-DLIBFEC_ENABLED")

find_library(FFTW3_LIBRARY
  NAMES ${CMAKE_STATIC_LIBRARY_PREFIX}fftw3f${CMAKE_STATIC_LIBRARY_SUFFIX}
  PATHS /usr/local/lib
        /usr/lib
)
list(APPEND _LIBRARIES ${FFTW3_LIBRARY})

find_library(fec_LIBRARY
    NAMES ${CMAKE_STATIC_LIBRARY_PREFIX}fec${CMAKE_STATIC_LIBRARY_SUFFIX}
    PATHS /usr/local/lib
          /usr/lib
)
list(APPEND _LIBRARIES ${fec_LIBRARY})

set_target_properties(liquid PROPERTIES
    INTERFACE_LINK_LIBRARIES "${_LIBRARIES}"
)

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

