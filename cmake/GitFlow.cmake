#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

# GitDescribe.cmake
#
# author: rschwarz@riegl.com
#
# "GitDescribe.cmake" and "git_describe.cmake.in" are two helper scripts to
# automate git description checking. The scripts generate a include file
# git_describe.hpp that contains the macro: GITDESCRIBE which expands to
# a string containing the result of "git describe --always --dirty".
#
# The unique feature of this script is that dirty checking is done on
# every build request, i.e., not only at configure time, while still
# avoiding unnecessary costly rebuild if nothing has changed.
# The scripts also will allow compilation when a) git is not installed or
# b) the sources are compiled outside of a git repository, however, with
# a default GITDESCRIBE "git-status-unavailable".
#
# Usage:
# put GitDescribe.cmake and git_describe.cmake.in in your source directory.
# In your CMakeLists.txt
# include(GitDescribe.cmake)
# Add git_describe.hpp to your sources and make use of the macro.

if (NOT DEFINED PROJECT_VERSION)
    message(FATAL_ERROR "PROJECT_VERSION must be defined.")
endif()

find_package(Git)
if (Git_FOUND)
    configure_file(${CMAKE_CURRENT_LIST_DIR}/git_flow.cmake.in git_flow.cmake @ONLY)
    add_custom_target(
        ${PROJECT_NAME}_git_flow ALL
        COMMAND ${CMAKE_COMMAND} -P git_flow.cmake
        BYPRODUCTS gitflow.hpp gitflow.cpp gitflow.txt
    )
else()
    message(WARNING "git is not available.")
endif()

configure_file ("${CMAKE_CURRENT_LIST_DIR}/git_flow_cpack.cmake.in" "git_flow_cpack.cmake" @ONLY)
set (CPACK_PROJECT_CONFIG_FILE "git_flow_cpack.cmake")
# write a default to the build directory, so that in case git is not
# available, the build still will run
file(WRITE ${PROJECT_BINARY_DIR}/gitflow.hpp
    [[namespace gitflow{extern const char* tag;}]]
)
file(WRITE ${PROJECT_BINARY_DIR}/gitflow.cpp
    [[namespace gitflow{const char* tag = "nogit";}]]
)
file(WRITE ${PROJECT_BINARY_DIR}/gitflow.txt
    "nogit"
)

add_library(gitflow STATIC
    ${PROJECT_BINARY_DIR}/gitflow.cpp
    ${PROJECT_BINARY_DIR}/gitflow.hpp
)

#set_property(TARGET gitflow PROPERTY POSITION_INDEPENDENT_CODE ON)

target_include_directories(gitflow PUBLIC
    ${PROJECT_BINARY_DIR}
)
# include_directories(
#     ${CMAKE_CURRENT_BINARY_DIR}
# )
