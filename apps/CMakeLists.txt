add_executable(lm2-gpio lm2-gpio.cpp)

target_link_libraries(lm2-gpio
    gitflow
    limesuiteng::limesuiteng
    spu::spu-static-lib
    CLI11::CLI11
)

install(TARGETS lm2-gpio
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

