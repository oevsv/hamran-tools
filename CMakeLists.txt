#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

cmake_minimum_required(VERSION 3.16)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")

project(hamran-tools LANGUAGES C CXX VERSION 0.0.4)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GitFlow)
include(GNUInstallDirs)

##
## Set up the external projects
##

add_subdirectory(extern)

##
## Set up the hamran-tools project
##

message(STATUS "Configure hamran-tools")
add_subdirectory(apps)

add_custom_target(doc SOURCES
    README.md
    CHANGELOG.md
)

include(CPack)
