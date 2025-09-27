/*
 * SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

/*
 * A tool for generating a beacon signal based on the work of Marek Honek
 * in his master thesis: SDR OFDM Frame Generation according to IEEE 802.22.
 * https://doi.org/10.34726/hss.2022.74390, but heavily adapted by OE1RSA.
 */

#include "gitflow.hpp"

#include <CLI/CLI.hpp>

#include <platform_folders.h>
using sago::getConfigHome;

#include <spdlog/spdlog.h>
using spdlog::trace, spdlog::debug, spdlog::info, spdlog::warn, spdlog::error,
    spdlog::critical;

#include <limesuiteng/limesuiteng.hpp>
using lime::DeviceRegistry, lime::DeviceHandle, lime::SDRDevice, lime::RFStream;

#include "boards/LimeSDR_Mini/LimeSDR_Mini.h"
using lime::LimeSDR_Mini;

#include <clocale>
using std::setlocale, std::locale;

#include <cstdlib>

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <thread>
using std::this_thread::sleep_for;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::duration_cast,
    std::chrono::nanoseconds, std::chrono::milliseconds, std::chrono::system_clock;
using namespace std::literals::chrono_literals;

#include <iostream>
using std::cout, std::endl;


#include <complex>
using std::complex, std::abs;
using namespace std::literals::complex_literals;

#include <memory>
using std::unique_ptr;

#include <vector>
using std::vector;

#include <string>
using std::string;

#include <format>
using std::format;

#include <exception>
using std::exception, std::invalid_argument;

#include <stdexcept>
using std::runtime_error;

namespace {

void limeSuiteLogHandler(const lime::LogLevel level, const string& message) {
    switch(level) {
    case lime::LogLevel::Critical: critical("lime: {}", message); break;
    case lime::LogLevel::Error:    error(   "lime: {}", message); break;
    case lime::LogLevel::Warning:  warn(    "lime: {}", message); break;
    case lime::LogLevel::Info:     info(    "lime: {}", message); break;
    case lime::LogLevel::Debug:    debug(   "lime: {}", message); break;
    case lime::LogLevel::Verbose:  trace(   "lime: {}", message); break;
    }
}

volatile sig_atomic_t signal_status = 0;
void signal_handler(int signal) {
    signal_status = signal;
}

}

int main(int argc, char** argv) {

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    int main_exit = EXIT_SUCCESS;
    CLI::App app{"hrbeacon: transmit a HAMRAN beacon."};
    argv = app.ensure_utf8(argv);

    DeviceHandle hint;
    string loglevel   = "info";
    LimeSDR_Mini* dev = nullptr;

    const double sample_rate = 2.4e6; // Hz

    // The following parameters can be configured at runtime
    double freq    = 53e6; // Hz
    double txpwr   =   11; // dBm
    double tone    =  1e3; // Hz
    double duration =  10; // s

    app.footer("Interactive mode assumed if no message given.");
    app.set_config("--config", getConfigHome() + "/hamran-tools/hrbeacon.ini");
    app.set_version_flag("--version", gitflow::tag);
    app.add_option("--serial,-s", hint.serial, "Load device serial");
    app.add_option("--level",      loglevel, "Specify the logging level")
        ->capture_default_str()
        ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));
    app.add_option("--freq, -f",    freq,   "Center frequency in Hz");
    app.add_option("--txpwr,-p",    txpwr, "Tx Pwr.of fullscale sine in dBm (-26dBm ... 10dBm)");
    app.add_option("--tone,-t",     tone, "Beacon test tone frequency in Hz");
    app.add_option("--duration,-d", duration, "Duration of beacon signals in seconds");

    try {
        app.parse(argc, argv);
        spdlog::set_level(spdlog::level::from_str(loglevel));
        spdlog::flush_on(spdlog::get_level());
        lime::registerLogHandler(limeSuiteLogHandler);

        info("{} {} started, loglevel is {}", app.get_name(), gitflow::tag, loglevel);

        // try to open the first lime mini matching "hint"
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
            info("Carrier frequency {:.2f} MHz", freq/1e6);
            info("TX power {:+.1f} dBm", txpwr);
            info("Test tone {:.3f} Hz", tone);
            info("Beacon duration {:.2f} s", duration);

            dev->SetMessageLogCallback(limeSuiteLogHandler);
            dev->Init();
            lime::SDRConfig cfg;
            {
                lime::ChannelConfig& ch{cfg.channel[0]};

                ch.rx.enabled = false;
                ch.rx.centerFrequency = freq;
                ch.rx.sampleRate = sample_rate;
                ch.rx.oversample = 2;
                ch.rx.lpf = 0;
                ch.rx.path = 3;
                ch.rx.calibrate = lime::CalibrationFlag::NONE;

                ch.tx.enabled = true;
                ch.tx.centerFrequency = freq;
                ch.tx.sampleRate = sample_rate;
                ch.tx.oversample = 2;
                ch.tx.path = 2;
                ch.tx.calibrate = lime::CalibrationFlag::NONE;
            }

            lime::StreamConfig strcfg;
            strcfg.channels[lime::TRXDir::Rx] = { 0 };
            strcfg.channels[lime::TRXDir::Tx] = { 0 };
            strcfg.format = lime::DataFormat::F32;
            strcfg.linkFormat = lime::DataFormat::I16;

            vector<complex<float>> tx_buffer(8192*4);
            vector<complex<float>> rx_buffer(8192*4);

            lime::StreamMeta rx_meta{};
            lime::StreamMeta tx_meta{};

            dev->Configure(cfg, 0);
            unique_ptr<RFStream> stream = dev->StreamCreate(strcfg, 0);
            stream->Start();

            debug("Streaming started");
            while (0 == signal_status) {
                lime::complex32f_t* rx_wrap[1] {reinterpret_cast<lime::complex32f_t*>(rx_buffer.data())};
                uint32_t rc = stream->StreamRx(rx_wrap, rx_buffer.size(), &rx_meta);
                if (rx_buffer.size() > rc) cout << "rx underrun" << endl;

                tx_meta.timestamp = rx_meta.timestamp + rx_buffer.size()*2;
                tx_meta.waitForTimestamp = true;
                tx_meta.flushPartialPacket = false;
                lime::complex32f_t* tx_wrap[1] {reinterpret_cast<lime::complex32f_t*>(tx_buffer.data())};
                uint32_t tc = stream->StreamTx(tx_wrap, tx_buffer.size(), &tx_meta);
                if (tx_buffer.size() > tc) cout << "tx overrun" << endl;

                //sleep_for(100ms);
            }

            stream->Stop();
            stream->Teardown();
            stream.reset();
        }



    } catch (const CLI::ParseError& e) {
        main_exit = app.exit(e);
    } catch (const exception& e) {
        critical("{}", e.what());
        main_exit = EXIT_FAILURE;
    } catch (...) {
        critical("Exception of unknown type");
        main_exit = EXIT_FAILURE;
    }

    if (nullptr != dev) {
        debug("closing device");
        DeviceRegistry::freeDevice(dev);
    }

    return main_exit;
}
