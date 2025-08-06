add_executable(lm2-gpio lm2-gpio.cpp)

target_link_libraries(lm2-gpio
    gitflow
    limesuiteng::limesuiteng
    liquid::liquid
    CLI11::CLI11
    cpp-terminal
    platform_folders
    spdlog
)

install(TARGETS lm2-gpio
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

