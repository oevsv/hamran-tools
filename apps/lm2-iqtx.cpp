//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "gitflow.hpp"

#include <CLI/CLI.hpp>

#include <platform_folders.h>
using sago::getConfigHome;

#include <spdlog/spdlog.h>
using spdlog::trace, spdlog::debug, spdlog::info, spdlog::warn, spdlog::error,
    spdlog::critical;

#include <limesuiteng/limesuiteng.hpp>
using lime::DeviceRegistry, lime::DeviceHandle, lime::SDRDevice;

#include "boards/LimeSDR_Mini/LimeSDR_Mini.h"
using lime::LimeSDR_Mini;

#include <string>
using std::string, std::stoi;

#include <iostream>
using std::cout, std::cerr, std::endl, std::ostream;

#include <exception>
using std::exception, std::invalid_argument;

#include <stdexcept>
using std::runtime_error;

// #include <complex> // NB: Must be included before liquid.h !
// using std::complex;
// #include <liquid.h>

namespace {

void limeSuiteLogHandler(const lime::LogLevel level, const string& message) {
    switch(level) {
    case lime::LogLevel::Critical: critical("lime: {}", message); break;
    case lime::LogLevel::Error:    error(   "lime {}",  message); break;
    case lime::LogLevel::Warning:  warn(    "lime: {}", message); break;
    case lime::LogLevel::Info:     info(    "lime: {}", message); break;
    case lime::LogLevel::Debug:    debug(   "lime: {}", message); break;
    case lime::LogLevel::Verbose:  trace(   "lime: {}", message); break;
    }
}

}

int main(int argc, char** argv) {

    CLI::App app{"lm2-iqrx: transmit an IQ file with lime miniv2"};
    argv = app.ensure_utf8(argv);

    DeviceHandle hint;
    string loglevel = "trace";

    app.footer("When no options are present the interactive mode is started.");
    app.set_config("--config", getConfigHome() + "/hamran-tools/lm2-iqtx.ini");
    app.set_version_flag("--version", gitflow::tag);
    app.add_option("--serial", hint.serial, "Load device serial");
    app.add_option("--level", loglevel, "Specify the logging level")
        ->capture_default_str()
        ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));

    LimeSDR_Mini* dev = nullptr;
    int main_exit = EXIT_SUCCESS;

    try{
        app.parse(argc, argv);
        spdlog::set_level(spdlog::level::from_str(loglevel));
        spdlog::flush_on(spdlog::get_level());
        lime::registerLogHandler(limeSuiteLogHandler);

        info("{} started, loglevel is {}", app.get_name(), loglevel);

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
            warn("could not open a device");
        else {
            dev->SetMessageLogCallback(limeSuiteLogHandler);
            dev->Init();
            lime::SDRConfig cfg;
            {
                lime::ChannelConfig& ch{cfg.channel[0]};

                ch.rx.enabled = false;
                ch.rx.centerFrequency = 53e6;
                ch.rx.sampleRate = 4e6;
                ch.rx.oversample = 2;
                ch.rx.lpf = 0;
                ch.rx.path = 3;
                ch.rx.calibrate = lime::CalibrationFlag::NONE;

                ch.tx.enabled = true;
                ch.tx.centerFrequency = 53e6;
                ch.tx.sampleRate = 4e6;
                ch.tx.oversample = 2;
                ch.tx.path = 2;
                ch.tx.calibrate = lime::CalibrationFlag::NONE;
            }
            lime::StreamConfig strcfg;

            strcfg.channels[lime::TRXDir::Rx] = { 0 };
            strcfg.channels[lime::TRXDir::Tx] = { 0 };
            strcfg.format = lime::DataFormat::F32;
            strcfg.linkFormat = lime::DataFormat::I16;

            dev->Configure(cfg, 0);

        }

    } catch (const CLI::ParseError& e) {
        main_exit = app.exit(e);
    } catch (const exception& e) {
        cerr << e.what() << endl;
        main_exit = EXIT_FAILURE;
    } catch (...) {
        cerr << "Exception of unknown type";
        main_exit = EXIT_FAILURE;
    }

    if (nullptr != dev) {
        debug("closing device");
        DeviceRegistry::freeDevice(dev);
    }

    return main_exit;
}
