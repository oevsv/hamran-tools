//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

/*
 * A test setup for estimating the tx/rx latency for the HAMRAN project.
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

#include <filesystem>
using std::filesystem::path, std::filesystem::create_symlink,
    std::filesystem::remove;

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

ostream &write(ostream &os, const vector<complex<float>> &data) {
  return os.write(reinterpret_cast<const char *>(data.data()),
                  sizeof(complex<float>) * data.size());
}

} // namespace

int main(int argc, char **argv) {

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  CLI::App app{"lm2-loop: Loopback delay estimation."};
  argv = app.ensure_utf8(argv);

  OpStatus ops;
  DeviceHandle hint;
  string loglevel = "info";
  path dstrx = "lm2-loop-rx";
  path dstrxlnk;
  path dsttx = "lm2-loop-tx";
  path dsttxlnk;

  const double sample_rate = 4e6; // Hz

  // The following parameters can be configured at runtime
  double freq = 53e6;   // Hz
  double tone = -500e3; // Hz
  double duration = 10; // s
  double lna = 30.0;    // dB, RX: Low Noise Amplifier gain
  double tia = 9.0;     // dB, RX: Trans Impedance Amplifier Gain
  double pga = 0.0;     // dB, RX: Programmable Gain Amplifier
  double iamp = 11.0;   // TX: Current Amplifier
  double pad = 40.0;    // dB, TX: Power Amplifier Driver

  app.set_config("--config", getConfigHome() + "/hamran-tools/lm2-loop.ini");
  app.set_version_flag("--version", gitflow::tag);
  app.add_option("--serial,-s", hint.serial, "Load device serial");
  app.add_option("--level", loglevel, "Specify the logging level")
      ->capture_default_str()
      ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));
  app.add_option("rxfilename", dstrx, "rx destination file")
      ->capture_default_str();
  app.add_option("txfilename", dsttx, "tx destination file")
      ->capture_default_str();
  app.add_option("--freq, -f", freq, "Center frequency in Hz");
  app.add_option("--tone,-t", tone, "Beacon test tone frequency in Hz");
  app.add_option("--duration,-d", duration,
                 "Duration of the measurement in seconds");
  app.add_option("--lna", lna, "RX: Low Noise Amplifier Gain 0 ... 30 dB")
      ->default_val(lna);
  app.add_option("--tia", tia, "RX: Trans Impedance Amplifier Gain [0,9,12] dB")
      ->default_val(tia);
  app.add_option("--pga", pga, "RX: Programmable Gain Amplifier -12 ... 19 dB")
      ->default_val(pga);
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
      dstrxlnk = dstrx;
      dstrxlnk.replace_extension(".c32_le");
      dstrx += format("_{:%Y-%m-%dT%H_%M_%S}.c32_le", start_time);
      dsttxlnk = dsttx;
      dsttxlnk.replace_extension(".c32_le");
      dsttx += format("_{:%Y-%m-%dT%H_%M_%S}.c32_le", start_time);
      info("Filename for rx saving: {}", dstrx.string());
      info("Filename for tx saving: {}", dsttx.string());
      info("Carrier frequency {:.2f} MHz", freq / 1e6);
      info("Test tone {:.3f} Hz", tone);
      info("Test duration {:.2f} s", duration);
      info("LNA: {:.1f} dB", lna);
      info("TIA: {:.1f} dB", tia);
      info("PGA: {:.1f} dB", pga);

      dev->SetMessageLogCallback(limeSuiteLogHandler);
      dev->Init();
      lime::SDRConfig cfg;
      {
        lime::ChannelConfig &ch{cfg.channel[0]};

        ch.rx.enabled = true;
        ch.rx.centerFrequency = freq;
        ch.rx.sampleRate = sample_rate;
        ch.rx.oversample = 0; // 1 << 5; // 32
        ch.rx.path = 3;       // RX1_W 0.03 - 1.9 GHz
        ch.rx.lpf = 2.5e6;
        ch.rx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.rx.gain[lime::eGainTypes::LNA] = lna; // 0 ... 30
        ch.rx.gain[lime::eGainTypes::TIA] = tia; // 0, 9, 12
        ch.rx.gain[lime::eGainTypes::PGA] = pga; // -12 ... 19
        ch.rx.gfir.enabled = true;
        ch.rx.gfir.bandwidth = 1.9e6;

        ch.tx.enabled = true;
        ch.tx.centerFrequency = freq;
        ch.tx.sampleRate = sample_rate;
        ch.tx.oversample = 0; // 1 << 5; // 32
        ch.tx.path = 2;       // TX1_2 0.03 - 1.9 GHz
        ch.tx.lpf = 5.5e6;
        ch.tx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.tx.gain[lime::eGainTypes::IAMP] = iamp; // 0 ... 63
        ch.tx.gain[lime::eGainTypes::PAD] = pad;   // 5 ... 55
        ch.tx.gfir.enabled = true;
        ch.tx.gfir.bandwidth = 1.9e6;
      }
      cfg.skipDefaults = false;

      cfg.channel[0].rx.sampleRate = sample_rate;
      cfg.channel[0].tx.sampleRate = sample_rate;
      ops = dev->Configure(cfg, 0);
      if (lime::OpStatus::Success != ops) {
        warn("configure returned nonzero status {:d}", unsigned(ops));
      }

      // complex<double> IQGain = dev->GetIQBalance(0, lime::TRXDir::Tx, 0);
      // info("IQGain: {:.3f}*exp({:.3f}j)", abs(IQGain), arg(IQGain));

      lime::StreamConfig strcfg;
      strcfg.channels[lime::TRXDir::Rx] = {0};
      strcfg.channels[lime::TRXDir::Tx] = {0};
      strcfg.format = lime::DataFormat::F32;
      strcfg.linkFormat = lime::DataFormat::I12;

      strcfg.extraConfig.rx.packetsInBatch = 8;
      strcfg.extraConfig.tx.packetsInBatch = 8;
      unique_ptr<RFStream> stream = dev->StreamCreate(strcfg, 0);
      stream->Start();

      vector<complex<float>> tx_buffer(2048);
      vector<complex<float>> rx_buffer(tx_buffer.size());

      StreamRxMeta rx_meta;
      StreamTxMeta tx_meta;

      complex<double> w = exp(2.0i * acos(-1) * double(tone / sample_rate));
      // Initialize the oscillator:
      complex<double> y = conj(w);

      ofstream outrx(dstrx, ios::out | ios::binary);
      remove(dstrxlnk);
      create_symlink(dstrx, dstrxlnk);

      ofstream outtx(dsttx, ios::out | ios::binary);
      remove(dsttxlnk);
      create_symlink(dsttx, dsttxlnk);

      info("Start streaming");

      double on = 1.0;
      size_t count_on = 0;
      size_t count_off = 0;
      while (0 == signal_status &&
             system_clock::now() < start_time + duration * 1s) {
        complex<float> *rx_wrap[1]{rx_buffer.data()};
        uint32_t rc = stream->Receive(rx_wrap, rx_buffer.size(), &rx_meta);
        if (rx_buffer.size() > rc)
          cout << "rx underrun" << endl;

        // tx_meta.timestamp =
        //     rx_meta.timestamp + 2 * rx_buffer.size() / sample_rate;
        tx_meta.hasTimestamp = false;
        tx_meta.flags = 0; // StreamTxMeta::EndOfBurst;

        for (size_t n = 0; n < tx_buffer.size(); ++n) {
          if (0.0 != on) {
            if (++count_on > size_t(5e-3 * sample_rate)) {
              count_on = 0;
              on = 0.0;
            }
          } else {
            if (++count_off > size_t(25e-3 * sample_rate)) {
              count_off = 0;
              on = 1.0;
            }
          }
          tx_buffer[n] = on * (y *= w);
        }

        complex<float> *tx_wrap[1]{tx_buffer.data()};
        uint32_t tc = stream->Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
        if (tx_buffer.size() > tc)
          cout << "tx overrun" << endl;

        // write(outrx, rx_buffer);
        // write(outtx, tx_buffer);
      }

      info("End streaming");

      outrx.close();
      outtx.close();

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
