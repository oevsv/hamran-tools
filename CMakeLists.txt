#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

cmake_minimum_required(VERSION 3.16)
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}")

project(hamran-tools LANGUAGES CXX VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(GitFlow)

#find_package(Threads REQUIRED)
#find_package(kissfft 131.1.0 REQUIRED)
find_package(limesuiteng 0.3.0 REQUIRED)
find_package(cpptrace REQUIRED)
#find_package(streampu REQUIRED)

add_executable(hamran-tool main.cpp)

target_link_libraries(hamran-tool
    gitflow
    limesuiteng::limesuiteng
    #spu::spu-static-lib
)

add_custom_target(doc SOURCES
    README.md
)

include(GNUInstallDirs)
install(TARGETS hamran-tool
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

include(CPack)
