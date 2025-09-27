#
# SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
#
# SPDX-License-Identifier: GPL-3.0-or-later
#

add_executable(lm2-gpio lm2-gpio.cpp)

target_link_libraries(lm2-gpio
    gitflow
    limesuiteng::limesuiteng
    CLI11::CLI11
    cpp-terminal
    platform_folders
    spdlog
)

add_executable(lm2-iqtx lm2-iqtx.cpp)

target_link_libraries(lm2-iqtx
    gitflow
    limesuiteng::limesuiteng
    liquid::liquid
    CLI11::CLI11
    cpp-terminal
    platform_folders
    spdlog
)

add_executable(lm2-iqrx lm2-iqrx.cpp)

target_link_libraries(lm2-iqrx
    gitflow
    limesuiteng::limesuiteng
    liquid::liquid
    CLI11::CLI11
    cpp-terminal
    platform_folders
    spdlog
)


add_executable(hrbeacon hrbeacon.cpp)

target_link_libraries(hrbeacon
    gitflow
    limesuiteng::limesuiteng
    liquid::liquid
    CLI11::CLI11
    platform_folders
    spdlog
)

install(TARGETS lm2-gpio lm2-iqrx lm2-iqtx hrbeacon
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

