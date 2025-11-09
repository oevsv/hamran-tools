//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

/*
 * Generate a complex sine for the HAMRAN project.
 */

#include "gitflow.hpp"

#include <CLI/CLI.hpp>

#include <platform_folders.h>
using sago::getConfigHome;

#include <spdlog/spdlog.h>
using spdlog::trace, spdlog::debug, spdlog::info, spdlog::warn, spdlog::error,
    spdlog::critical;

#include <limesuiteng/limesuiteng.hpp>
using lime::DeviceRegistry, lime::DeviceHandle, lime::SDRDevice, lime::RFStream,
    lime::OpStatus;

#include "limesuiteng/StreamMeta.h"
using lime::StreamRxMeta, lime::StreamTxMeta;

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
    std::chrono::nanoseconds, std::chrono::milliseconds,
    std::chrono::system_clock;
using namespace std::literals::chrono_literals;

#include <iostream>
using std::cout, std::endl, std::ostream;

#include <fstream>
using std::ofstream, std::ios;

#include <complex>
using std::complex, std::abs, std::conj, std::exp;
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

void limeSuiteLogHandler(const lime::LogLevel level, const string &message) {
  switch (level) {
  case lime::LogLevel::Critical:
    critical("lime: {}", message);
    break;
  case lime::LogLevel::Error:
    error("lime: {}", message);
    break;
  case lime::LogLevel::Warning:
    warn("lime: {}", message);
    break;
  case lime::LogLevel::Info:
    info("lime: {}", message);
    break;
  case lime::LogLevel::Debug:
    debug("lime: {}", message);
    break;
  case lime::LogLevel::Verbose:
    trace("lime: {}", message);
    break;
  }
}

volatile sig_atomic_t signal_status = 0;
void signal_handler(int signal) { signal_status = signal; }

} // namespace

int main(int argc, char **argv) {

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  CLI::App app{"lm2-loop: Loopback delay estimation."};
  argv = app.ensure_utf8(argv);

  OpStatus ops;
  DeviceHandle hint;
  string loglevel = "info";

  const double sample_rate = 4e6;  // Hz
  const double txant_pre = 37e-6;  // s
  const double txant_post = 37e-6; // s

  // The following parameters can be configured at runtime
  double freq = 53e6;   // Hz
  double tone = 0;      // 500e3;  // Hz
  double duration = 10; // s
  bool pulsed = false;
  double iamp = 11.0; // TX: Current Amplifier
  double pad = 40.0;  // dB, TX: Power Amplifier Driver

  app.set_config("--config", getConfigHome() + "/hamran-tools/lm2-singen.ini");
  app.set_version_flag("--version", gitflow::tag);
  app.add_option("--serial,-s", hint.serial, "Load device serial");
  app.add_option("--level", loglevel, "Specify the logging level")
      ->capture_default_str()
      ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));
  app.add_option("--freq, -f", freq, "Center frequency in Hz");
  app.add_option("--tone,-t", tone, "Beacon test tone frequency in Hz");
  app.add_option("--duration,-d", duration,
                 "Duration of the measurement in seconds");
  app.add_flag("--pulsed,-p", pulsed, "turn on pulse modulation");
  app.add_option("--iamp", iamp, "TX: Current Amplifier 0 ... 63")
      ->default_val(iamp);
  app.add_option("--pad", pad, "TX: Power Amplifier Driver 5 ... 55")
      ->default_val(pad);

  LimeSDR_Mini *dev = nullptr;
  int main_exit = EXIT_SUCCESS;

  try {
    app.parse(argc, argv);
    spdlog::set_level(spdlog::level::from_str(loglevel));
    spdlog::flush_on(spdlog::get_level());
    lime::registerLogHandler(limeSuiteLogHandler);

    info("{} {} started, loglevel is {}", app.get_name(), gitflow::tag,
         loglevel);

    // try to open the first lime mini matching "hint"
    debug("device hint is: [{}]", hint.Serialize());
    for (auto &dh : DeviceRegistry::enumerate(hint)) {
      debug("trying to open {}, {}", dh.name, dh.serial);
      SDRDevice *d = DeviceRegistry::makeDevice(dh);
      if (nullptr != d) {
        dev = dynamic_cast<LimeSDR_Mini *>(d);
        if (nullptr == dev) {
          debug("device is not a LimeSDR Mini");
          DeviceRegistry::freeDevice(d);
        } else {
          info("device opened: {}", dh.Serialize());
          break;
        }
      }
    }
    if (nullptr == dev)
      warn("could not open a device");
    else {
      auto start_time = system_clock::now();
      info("Carrier frequency {:.2f} MHz", freq / 1e6);
      info("Test tone {:.3f} Hz", tone);
      info("Test duration {:.2f} s", duration);

      dev->SetMessageLogCallback(limeSuiteLogHandler);
      dev->Init();

      // set up the fast, data synchronous RX/TX signaling
      // https://github.com/myriadrf/LimeSDR-Mini-v2_GW/issues/3
      // https://discourse.myriadrf.org/t/limemini-2-2-gpio/8012/5

      uint16_t gpioctl = dev->ReadRegister(0, 0x00c0, true);
      gpioctl &= 0xfffe; // clear bit 0 to enable PA ctrl
      dev->WriteRegister(0, 0x00c0, gpioctl, true);
      dev->WriteRegister(0, 0x0010,
                         static_cast<uint16_t>(txant_pre * sample_rate), true);
      dev->WriteRegister(0, 0x0011,
                         static_cast<uint16_t>(txant_post * sample_rate), true);

      lime::SDRConfig cfg;
      {
        lime::ChannelConfig &ch{cfg.channel[0]};

        ch.rx.enabled = false;
        ch.rx.centerFrequency = freq;
        ch.rx.sampleRate = sample_rate;

        ch.tx.enabled = true;
        ch.tx.centerFrequency = freq;
        ch.tx.sampleRate = sample_rate;
        ch.tx.oversample = 1 << 5; // 32
        ch.tx.path = 2;            // TX1_2 0.03 - 1.9 GHz
        ch.tx.lpf = 5.5e6;
        ch.tx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.tx.gain[lime::eGainTypes::IAMP] = iamp; // 0 ... 63
        ch.tx.gain[lime::eGainTypes::PAD] = pad;   // 5 ... 55
        ch.tx.gfir.enabled = true;
        ch.tx.gfir.bandwidth = 1.9e6;
      }
      cfg.skipDefaults = false;

      ops = dev->Configure(cfg, 0);
      if (lime::OpStatus::Success != ops) {
        warn("configure returned nonzero status {:d}", unsigned(ops));
      }

      lime::StreamConfig strcfg;
      strcfg.channels[lime::TRXDir::Tx] = {0};
      strcfg.format = lime::DataFormat::F32;
      strcfg.linkFormat = lime::DataFormat::I12;

      strcfg.extraConfig.tx.packetsInBatch = 8;
      strcfg.extraConfig.rx.packetsInBatch = 8;
      unique_ptr<RFStream> stream = dev->StreamCreate(strcfg, 0);
      stream->Start();

      vector<complex<float>> tx_buffer(1024);

      StreamTxMeta tx_meta;

      complex<double> w = exp(2.0i * acos(-1) * double(tone / sample_rate));
      // Initialize the oscillator:
      complex<double> y = conj(w);

      const size_t frame_ticks = sample_rate * 10e-3;
      const size_t frame_size = frame_ticks / 2;

      info("Start streaming");

      while (0 == signal_status &&
             system_clock::now() < start_time + duration * 1s) {

        tx_meta.timestamp.AddTicks(frame_ticks);
        tx_meta.hasTimestamp = pulsed;
        tx_meta.flags = 0;

        size_t pending_size = frame_size;
        size_t count;
        while (pending_size > tx_buffer.size()) {
          for (size_t n = 0; n < tx_buffer.size(); ++n)
            tx_buffer[n] = (y *= w);
          complex<float> *tx_wrap[1]{tx_buffer.data()};
          count = stream->Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
          if (0 == count) {
            warn("queue full, waiting for 100ms");
            sleep_for(100ms);
            if (0 != signal_status)
              break;
          }
          pending_size -= count;
          tx_meta.hasTimestamp = false;
        }

        for (size_t n = 0; n < pending_size; ++n)
          tx_buffer[n] = 1.0 * (y *= w);

        tx_meta.flags = StreamTxMeta::EndOfBurst;
        while (pending_size > 0) {
          complex<float> *tx_wrap[1]{tx_buffer.data()};
          count = stream->Transmit(tx_wrap, pending_size, &tx_meta);
          if (0 == count) {
            warn("queue full, waiting for 100ms");
            sleep_for(100ms);
            if (0 != signal_status)
              break;
          }
          pending_size -= count;
        }
      }

      info("End streaming");

      stream->Stop();
      stream->Teardown();
      stream.reset();
      dev->Reset();
    }

  } catch (const CLI::ParseError &e) {
    main_exit = app.exit(e);
  } catch (const exception &e) {
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
