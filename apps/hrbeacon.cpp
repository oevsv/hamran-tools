//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

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
using std::jthread, std::stop_token, std::this_thread::sleep_for;

#include <mutex>
using std::mutex;

#include <atomic>
using std::atomic;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::duration_cast,
    std::chrono::nanoseconds, std::chrono::milliseconds,
    std::chrono::system_clock;
using namespace std::literals::chrono_literals;

#include <iostream>
using std::cout, std::cin, std::endl, std::flush, std::ostream;

#include <complex>
using std::complex, std::abs, std::conj, std::exp;
using namespace std::literals::complex_literals;

#include <memory>
using std::unique_ptr;

#include <vector>
using std::vector;

#include <string>
using std::string, std::getline;

#include <algorithm>
using std::min;

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

const double sample_rate = 4e6;  // Hz
const double txant_pre = 37e-6;  // s
const double txant_post = 37e-6; // s

string message; // the beacon text
mutex message_mutex;
atomic<bool> is_interactive = true;

void transmit(stop_token stoken, RFStream *tx_stream, double tone) {
  debug("tx thread started");
  const size_t frame_ticks = sample_rate * 10e-3;

  vector<complex<float>> tx_buffer(1024);
  StreamTxMeta tx_meta;

  complex<double> w = exp(2.0i * acos(-1) * double(tone / sample_rate));
  // Initialize the oscillator:
  complex<double> y = conj(w);

  const size_t frame_size = 20'000;
  tx_meta.timestamp.AddTicks(1000 * frame_ticks);

  while (!stoken.stop_requested()) {

    tx_meta.timestamp.AddTicks(2 * frame_ticks);

    // cout << format("{:.2f} {:.2f}", tx_meta.timestamp.GetTicks() /
    // sample_rate,
    //                tx_stream->GetHardwareTimestamp() / sample_rate)
    //      << endl;

    tx_meta.hasTimestamp = true;
    tx_meta.flags = 0;

    size_t pending_size = frame_size;
    size_t count;
    while (pending_size > tx_buffer.size()) {
      for (size_t n = 0; n < tx_buffer.size(); ++n)
        tx_buffer[n] = 1.0 * (y *= w);
      complex<float> *tx_wrap[1]{tx_buffer.data()};
      count = tx_stream->Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
      if (0 == count) {
        warn("queue full, waiting for 100ms");
        sleep_for(100ms);
        if (stoken.stop_requested())
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
      count = tx_stream->Transmit(tx_wrap, pending_size, &tx_meta);
      if (0 == count) {
        warn("queue full, waiting for 100ms");
        sleep_for(100ms);
        if (stoken.stop_requested())
          break;
      }
      pending_size -= count;
    }
  }

  debug("tx thread stopping");
}

void receive(stop_token stoken, RFStream *rx_stream) {
  debug("rx thread started");

  vector<complex<float>> rx_buffer(1024);
  StreamRxMeta rx_meta;

  while (!stoken.stop_requested()) {
    complex<float> *rx_wrap[1]{rx_buffer.data()};
    uint32_t rc = rx_stream->Receive(rx_wrap, rx_buffer.size(), &rx_meta);
    if (rx_buffer.size() > rc)
      cout << "rx underrun" << endl;
  }

  debug("rx thread stopping");
}

int main(int argc, char **argv) {

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  CLI::App app{"hrbeacon: transmit a HAMRAN beacon."};
  argv = app.ensure_utf8(argv);

  OpStatus ops;
  DeviceHandle hint;
  string loglevel = "info";

  // The following parameters can be configured at runtime
  double freq = 53e6;   // Hz
  double txpwr = 11;    // dBm
  double tone = 0;      // Hz
  double duration = 10; // s
  double lna = 30.0;    // dB, RX: Low Noise Amplifier gain
  double tia = 9.0;     // dB, RX: Trans Impedance Amplifier Gain
  double pga = 0.0;     // dB, RX: Programmable Gain Amplifier
  double iamp = 11.0;   // TX: Current Amplifier
  double pad = 40.0;    // dB, TX: Power Amplifier Driver

  app.footer("Interactive mode assumed if no message given.");
  app.set_config("--config", getConfigHome() + "/hamran-tools/hrbeacon.ini");
  app.set_version_flag("--version", gitflow::tag);
  app.add_option("--serial,-s", hint.serial, "Load device serial");
  app.add_option("--level", loglevel, "Specify the logging level")
      ->capture_default_str()
      ->check(CLI::IsMember(SPDLOG_LEVEL_NAMES));
  app.add_option("--freq, -f", freq, "Center frequency in Hz");
  app.add_option("--txpwr,-p", txpwr,
                 "Tx Pwr.of fullscale sine in dBm (-26dBm ... 10dBm)");
  app.add_option("--tone,-t", tone, "Beacon test tone frequency in Hz");
  app.add_option("--duration,-d", duration,
                 "Duration of beacon signals in seconds");
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
  app.add_option("message", message, "beacon message text");

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
          lime::SDRDescriptor descr = dev->GetDescriptor();
          info("firmwareVersion: {}", descr.firmwareVersion);
          info("gatewareVersion: {}", descr.gatewareVersion);
          info("gateware.Revision: {}", descr.gatewareRevision);
          info("gatewareTargetBoard: {}", descr.gatewareTargetBoard);
          info("hardwareVersion: {}", descr.hardwareVersion);
          info("protocolVersion: {}", descr.protocolVersion);
          break;
        }
      }
    }
    if (nullptr == dev)
      warn("could not open a device");
    else {
      info("Carrier frequency {:.2f} MHz", freq / 1e6);
      info("TX power {:+.1f} dBm", txpwr);
      info("Test tone {:.3f} Hz", tone);
      info("Beacon duration {:.2f} s", duration);
      info("LNA: {:.1f} dB", lna);
      info("TIA: {:.1f} dB", tia);
      info("PGA: {:.1f} dB", pga);

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

        ch.rx.enabled = true;
        ch.rx.centerFrequency = freq;
        ch.rx.sampleRate = sample_rate;
        ch.rx.oversample = 1 << 5; // 32
        ch.rx.path = 3;            // RX1_W 0.03 - 1.9 GHz
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
        ch.tx.oversample = 1 << 5; // 32
        ch.tx.path = 2;            // TX1_2 0.03 - 1.9 GHz
        ch.tx.lpf = 5.5e6;
        ch.tx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.tx.gain[lime::eGainTypes::IAMP] = iamp; // 24.0; // 0 ... 63
        ch.tx.gain[lime::eGainTypes::PAD] = pad;   // 44.0;  // 5 ... 55
        ch.tx.gfir.enabled = true;
        ch.tx.gfir.bandwidth = 1.9e6;
      }
      cfg.skipDefaults = false;

      ops = dev->Configure(cfg, 0);
      if (lime::OpStatus::Success != ops) {
        warn("configure returned nonzero status {:d}", unsigned(ops));
      }

      complex<double> IQGain = dev->GetIQBalance(0, lime::TRXDir::Tx, 0);
      info("IQGain: {:.3f}*exp({:.3f}j)", abs(IQGain), arg(IQGain));

      lime::StreamConfig strcfg;
      strcfg.channels[lime::TRXDir::Rx] = {0};
      strcfg.channels[lime::TRXDir::Tx] = {0};
      strcfg.format = lime::DataFormat::F32;
      strcfg.linkFormat = lime::DataFormat::I12;

      strcfg.extraConfig.rx.packetsInBatch = 8;
      strcfg.extraConfig.tx.packetsInBatch = 8;
      unique_ptr<RFStream> stream = dev->StreamCreate(strcfg, 0);
      stream->Start();

      info("Start streaming");
      if (message.empty()) {
        is_interactive = true;
        message = "OE1XDU The quick brown fox jumps over the lazy dog.";
      } else {
        info("Beacon text: {}", message);
        is_interactive = false;
      }

      jthread rx_thread(receive, stream.get());
      jthread tx_thread(transmit, stream.get(), tone);

      if (is_interactive) {
        cout << "Beacon control thread: interactive mode, enter message or EOF "
                "(Ctrl-D) or empty "
                "line to end."
             << endl;
        cout << "Default text is: " << message << endl;
        string line;
        cout << "> " << flush;
        while (getline(cin, line)) {
          if (line.empty())
            break;
          message_mutex.lock();
          message = line;
          message_mutex.unlock();
          cout << "> " << flush;
        }
      } else {
        info("Beaconcontrol thread running: stop with SIGINT (Ctrl-C) or "
             "SIGTERM.");
        while (0 == signal_status) {
          sleep_for(100ms);
        }
        info("Caught signal {}, exiting ...", signal_status);
      }

      // vector<complex<float>> tx_buffer(1024);
      // vector<complex<float>> rx_buffer(tx_buffer.size());

      // StreamRxMeta rx_meta;
      // StreamTxMeta tx_meta;

      // complex<double> w = exp(2.0i * acos(-1) * double(tone / sample_rate));
      // // Initialize the oscillator:
      // complex<double> y = conj(w);

      // double on = 1.0;
      // size_t count_on = 0;
      // size_t count_off = 0;
      // // while (0 == signal_status) {
      //   complex<float> *rx_wrap[1]{rx_buffer.data()};
      //   uint32_t rc = stream->Receive(rx_wrap, rx_buffer.size(), &rx_meta);
      //   if (rx_buffer.size() > rc)
      //     cout << "rx underrun" << endl;

      //   tx_meta.timestamp =
      //       rx_meta.timestamp + 2 * rx_buffer.size() / sample_rate;
      //   tx_meta.hasTimestamp = false;
      //   tx_meta.flags = 0; // StreamTxMeta::EndOfBurst;

      //   for (size_t n = 0; n < tx_buffer.size(); ++n) {
      //     if (0.0 != on) {
      //       if (++count_on > size_t(5e-3 * sample_rate)) {
      //         count_on = 0;
      //         on = 0.0;
      //       }
      //     } else {
      //       if (++count_off > size_t(25e-3 * sample_rate)) {
      //         count_off = 0;
      //         on = 1.0;
      //       }
      //     }
      //     tx_buffer[n] = on * (y *= w);
      //   }

      //   complex<float> *tx_wrap[1]{tx_buffer.data()};
      //   uint32_t tc = stream->Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
      //   if (tx_buffer.size() > tc)
      //     cout << "tx overrun" << endl;
      // }

      tx_thread.request_stop();
      tx_thread.join();

      rx_thread.request_stop();
      rx_thread.join();

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
