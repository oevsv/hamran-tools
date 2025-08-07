//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "gitflow.hpp"

#include <CLI/CLI.hpp>

#include <cpp-terminal/terminal.hpp>
using Term::terminal;

#include <cpp-terminal/tty.hpp>
using Term::is_stdin_a_tty;

#include <cpp-terminal/input.hpp>
using Term::Key, Term::read_event;

#include <platform_folders.h>
using sago::getConfigHome;

#include <spdlog/spdlog.h>
using spdlog::debug, spdlog::info;

#include "FPGA/FPGA_common.h"
#include "boards/LimeSDR_Mini/LimeSDR_Mini.h"
#include <limesuiteng/limesuiteng.hpp>
using lime::DeviceRegistry, lime::DeviceHandle, lime::SDRDevice, lime::LimeSDR_Mini;


#include <iostream>
using std::cout, std::cerr, std::endl, std::ostream;

#include <exception>
using std::exception, std::invalid_argument;

#include <stdexcept>
using std::runtime_error;

#include <cstddef>
using std::byte, std::to_integer;;

#include <bitset>
using std::bitset;

#include <vector>
using std::vector;

#include <string>
using std::string, std::stoi;

ostream& operator<<(ostream& os, byte b) {
    return os << bitset<8>(to_integer<int>(b));
}

int main(int argc, char** argv)
{

    CLI::App app{"lm2-gpio: control GPIO lines of lime miniv2"};
    argv = app.ensure_utf8(argv);

    bool list = false;
    DeviceHandle hint;
    string loglevel = "trace";

    app.footer("When no options are present the interactive mode is started.");
    app.set_config("--config", getConfigHome() + "/hamran-tools/lm2-gpio.ini");
    app.set_version_flag("--version", gitflow::tag);
    app.add_flag("-l,--list", list, "List Devices and exit");
    app.add_option("--name", hint.name, "Load device name");
    app.add_option("--serial", hint.serial, "Load device serial");
    app.add_option("--level", loglevel, "Specify the logging level")
        ->capture_default_str()
        ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));

    try {
        app.parse(argc, argv);
        spdlog::set_level(spdlog::level::from_str(loglevel));
        spdlog::flush_on(spdlog::get_level());

        info("{} started", app.get_name());
        info("loglevel: {}", loglevel);

        if (list) {
            for (auto& dh : DeviceRegistry::enumerate())
                cout << dh.Serialize() << endl;
            return EXIT_SUCCESS;
        }

        LimeSDR_Mini* dev = nullptr;
        debug("device hint is: [{}]", hint.Serialize());
        for (auto& dh : DeviceRegistry::enumerate(hint)) {
            debug("trying to open {}, {}", dh.name, dh.serial);
            SDRDevice* d = DeviceRegistry::makeDevice(dh);
            if (nullptr != d) {
                dev = dynamic_cast<LimeSDR_Mini*>(d);
                if (nullptr == dev) {
                    debug("device is not a LimeSDR Mini");
                    DeviceRegistry::freeDevice(d);
                } else
                    break;
            }
        }
        if (nullptr == dev)
            debug("could not open a device");
        else {
            terminal.setOptions(Term::Option::Raw);
            if (!is_stdin_a_tty())
                throw runtime_error("no interactive mode, not atty");

            cout << "Press 0..7 to toggle pin, q to end." << endl;

            uint16_t fpga = dev->GetFPGA()->ReadRegister(0x00c0);
            // force all GPIO to user override
            fpga |= 0xff;
            dev->GetFPGA()->WriteRegister(0x00c0, fpga);

            uint8_t gpio_dir{0xff};
            dev->GPIODirWrite(&gpio_dir, 1);
            gpio_dir = 0x00;
            dev->GPIODirRead(&gpio_dir, 1);

            uint8_t gpio{0};
            dev->GPIORead(&gpio, 1);
            cout << byte(gpio) << endl;
            Key key(read_event());
            while (Term::Key::q != key) {
                try {
                    int bit = stoi(key.name(), nullptr, 8);
                    if (0 <= bit && bit <= 7) {
                        gpio ^= uint8_t(1 << bit);
                        dev->GPIOWrite(&gpio, 1);
                    }
                } catch (invalid_argument) {
                    /* expected for non octal keys */
                }
                dev->GPIORead(&gpio, 1);
                cout << byte(gpio) << endl;
                key = Key(read_event());
            }

            debug("closing device");
            DeviceRegistry::freeDevice(dev);
        }

    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    } catch (const exception& e) {
        cerr << e.what() << endl;
    } catch (...) {
        cerr << "Exception of unknown type";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
