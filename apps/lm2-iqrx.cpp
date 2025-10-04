//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "gitflow.hpp"
#include "limesuiteng/StreamMeta.h"

#include <CLI/CLI.hpp>

#include <platform_folders.h>
using sago::getConfigHome;

#include <spdlog/spdlog.h>
using spdlog::trace, spdlog::debug, spdlog::info, spdlog::warn, spdlog::error,
    spdlog::critical;

#include <limesuiteng/limesuiteng.hpp>
using lime::DeviceRegistry, lime::DeviceHandle, lime::SDRDevice, lime::RFStream,
    lime::OpStatus;

#include "boards/LimeSDR_Mini/LimeSDR_Mini.h"
using lime::LimeSDR_Mini;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <memory>
using std::unique_ptr;

#include <vector>
using std::vector;

#include <string>
using std::string, std::stoi;

#include <format>
using std::format;

#include <iostream>
using std::cout, std::cerr, std::endl, std::ostream;

#include <fstream>
using std::ofstream, std::ios;

#include <filesystem>
using std::filesystem::path;

#include <exception>
using std::exception, std::invalid_argument;

#include <stdexcept>
using std::runtime_error;

#include <complex>
using std::complex, std::abs;
using namespace std::literals::complex_literals;

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

volatile sig_atomic_t signal_status = 0;
void signal_handler(int signal) {
    signal_status = signal;
}

}

int main(int argc, char** argv) {

    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    CLI::App app{"lm2-iqrx: receive an IQ file with lime miniv2"};
    argv = app.ensure_utf8(argv);

    OpStatus os;
    DeviceHandle hint;
    string loglevel = "info";
    path dst;

    double sample_rate =  4e6; // Hz
    double freq        = 53e6; // Hz

    //app.footer("When no options are present the interactive mode is started.");
    app.set_config("--config", getConfigHome() + "/hamran-tools/lm2-iqtx.ini");
    app.set_version_flag("--version", gitflow::tag);
    app.add_option("--serial", hint.serial, "Load device serial");
    app.add_option("--level", loglevel, "Specify the logging level")
        ->capture_default_str()
        ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));
    app.add_option("filename", dst, "destination file")
        ->required();
    app.add_option("--freq,-f", freq, "Center frequency in Hz")
        ->default_val(freq);
    app.add_option("--sample_rate,-s", sample_rate, "Sample rate")
        ->default_val(sample_rate);

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
            dst.replace_extension(".c32_le");
            info("Filename for saving: {}", dst.string());
            info("Carrier frequency {:.2f} MHz", freq/1e6);
            info("Sample rate {:.3f} MHz", sample_rate/1e6);

            dev->SetMessageLogCallback(limeSuiteLogHandler);
            dev->Init();
            lime::SDRConfig cfg;
            {
                lime::ChannelConfig& ch{cfg.channel[0]};

                ch.rx.enabled = true;
                ch.rx.centerFrequency = freq;
                ch.rx.sampleRate = sample_rate;
                ch.rx.oversample = 2;
                ch.rx.lpf = 0;
                ch.rx.path = 3;
                ch.rx.calibrate = lime::CalibrationFlag::NONE;
                ch.rx.testSignal.enabled = false;
                ch.rx.gain[lime::eGainTypes::LNA] = 30.0;
                // ch.rx.gain[lime::eGainTypes::TIA] = 1.0;
                // ch.rx.gain[lime::eGainTypes::PGA] = 1.0;
                // ch.rx.gain[lime::eGainTypes::PAD] = 1.0;
                // ch.rx.gain[lime::eGainTypes::IAMP] = 1.0;
                // ch.rx.gain[lime::eGainTypes::PA] = 1.0;

                ch.tx.enabled = true;
                ch.tx.centerFrequency = freq;
                ch.tx.sampleRate = sample_rate;
                ch.tx.oversample = 3;
                ch.tx.path = 2;
                ch.tx.calibrate = lime::CalibrationFlag::NONE;
                ch.tx.testSignal.enabled = false;
            }
            lime::StreamConfig strcfg;

            strcfg.channels[lime::TRXDir::Rx] = { 0 };
            strcfg.channels[lime::TRXDir::Tx] = { 0 };
            strcfg.format = lime::DataFormat::F32;
            strcfg.linkFormat = lime::DataFormat::I16;

            strcfg.extraConfig.rx.packetsInBatch = 8;

            vector<complex<float>> rx_buffer(2048);
            lime::StreamRxMeta rx_meta;

            os = dev->Configure(cfg, 0);
            if (lime::OpStatus::Success != os) {
                cout << unsigned(os) << endl;
            }
            //os = dev->SetGain(0, lime::TRXDir::Rx, 3, lime::eGainTypes::LNA, 30);

            unique_ptr<RFStream> stream = dev->StreamCreate(strcfg, 0);
            stream->Start();

            uint64_t timestamp_expected = 0;
            uint64_t timestamp_anchor = 0;
            //lime::StreamStats rx_stat;
            complex<float> last_sample;
            ofstream out(dst, ios::out|ios::binary);
            info("Start streaming");
            while (0 == signal_status) {
                lime::complex32f_t* rx_wrap[1] {reinterpret_cast<lime::complex32f_t*>(rx_buffer.data())};
                uint32_t rc = stream->Receive(rx_wrap, rx_buffer.size(), &rx_meta);
                //out.write(reinterpret_cast<char*>(rx_buffer.data()), sizeof(complex<float>)*rx_buffer.size());
                if (rx_buffer.size() > rc) cout << "rx underrun" << endl;
                uint64_t rx_timestamp = rx_meta.timestamp.GetTicks();
                if (timestamp_expected > rx_timestamp) {
                    cout << format("TS: {}, expected: +{}\n", rx_timestamp, timestamp_expected - rx_timestamp);
                    if (rx_buffer.front() == last_sample) {
                        cout << "samples match" << endl;
                    }
                } else if (timestamp_expected < rx_timestamp) {
                    cout << format("TS: {}, expected: -{}, length: {}\n", rx_timestamp, rx_timestamp - timestamp_expected, timestamp_expected - timestamp_anchor);
                    timestamp_anchor = rx_timestamp;
                 }
                timestamp_expected = rx_timestamp + rx_buffer.size();
                 last_sample = rx_buffer.back();
            }
            out.close();
            info("stopping");

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
