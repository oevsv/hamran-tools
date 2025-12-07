//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

/*
 * A tool for generating a beacon signal based on the work of Marek Honek
 * in his master thesis: SDR OFDM Frame Generation according to IEEE 802.22.
 * https://doi.org/10.34726/hss.2022.74390, but adapted by OE1RSA.
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

#include <limesuiteng/StreamMeta.h>
using lime::StreamRxMeta, lime::StreamTxMeta;

#include <boards/LimeSDR_Mini/LimeSDR_Mini.h>
using lime::LimeSDR_Mini;

#include "limeutil.hpp"
using lime::util::limeSuiteLogHandler, lime::util::stream_clock;

#include "grcudp.hpp"
#include "hamranfrm.hpp"
#include "keyer.hpp"

#include <clocale>
using std::setlocale, std::locale;

#include <cstdint>
#include <cstdlib>

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <thread>
using std::jthread, std::stop_token, std::this_thread::sleep_for;

#include <mutex>
using std::mutex;

#include <atomic>
using std::atomic;

#include <ratio>
using std::ratio;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::system_clock,
    std::chrono::duration_cast, std::chrono::nanoseconds,
    std::chrono::milliseconds, std::chrono::microseconds,
    std::chrono::system_clock, std::chrono::time_point_cast, std::chrono::ceil,
    std::chrono::duration, std::chrono::clock_cast;
using ten_milliseconds = std::chrono::duration<int64_t, ratio<1, 100>>;
using namespace std::literals::chrono_literals;

#include <iostream>
using std::cout, std::cin, std::endl, std::flush, std::ostream;

#include <cmath>
using std::sqrt;

#include <complex>
using std::complex, std::abs, std::conj, std::exp;
using namespace std::literals::complex_literals;

#include <memory>
using std::unique_ptr;

#include <vector>
using std::vector;

#include <format>
using std::format;

#include <string>
using std::string, std::getline;

#include <algorithm>
using std::min;

#include <functional>
using std::ref;

#include <format>
using std::format;

#include <exception>
using std::exception, std::invalid_argument;

#include <stdexcept>
using std::runtime_error;

namespace {

volatile sig_atomic_t signal_status = 0;
void signal_handler(int signal) { signal_status = signal; }

} // namespace

const uint64_t sample_rate = 4'000'000; // Hz
const double txant_pre = 37e-6;         // s
const double txant_post = 37e-6;        // s

string global_message; // the beacon text
mutex message_mutex;

void transmit(stop_token stoken, RFStream &tx_stream, double tone,
              double duration, bool is_interactive, size_t cpf,
              size_t phy_mode) {

  debug("tx thread started");

  stream_clock<sample_rate> sclock(tx_stream);

  // Set up the OFDM frame
  unsigned char header[8] = {0, 0, 0, 0, 0, 0, 0, 0};
  hrframegen fg(sample_rate, -11, cpf, phy_mode);

  double sinamp = sqrt(0.5);
  // Set up the CW keyer
  keyer k(80, tone, 0, sinamp, sample_rate);

  // Set up the oscillator
  complex<double> w = exp(2.0i * acos(-1) * double(tone / sample_rate));
  complex<double> y = conj(w);

  // const size_t frame_size = 20'000;

  // The transmit buffer
  vector<complex<float>> tx_buffer(fg.subcarriers + fg.prefix_len);
  StreamTxMeta tx_meta;

  // The total cycle counter
  unsigned int count = 0;

  // Message captured and transfered to thread
  string message;

  while (!stoken.stop_requested()) {
    if (!is_interactive) {
      info("count: {:6d}", ++count);
    }

    // Capture the message
    message_mutex.lock();
    message = global_message;
    message_mutex.unlock();

    k.send(message);

    tx_meta.hasTimestamp = false;
    tx_meta.flags = 0;

    size_t total_samples = 0;

    // Key the CW signal
    size_t size = k.get_frame(tx_buffer.data(), tx_buffer.size());
    while (0 < size && !stoken.stop_requested()) {
      while (0 < size && !stoken.stop_requested()) {
        complex<float> *tx_wrap[1]{tx_buffer.data()};
        size -= tx_stream.Transmit(tx_wrap, size, &tx_meta);
      }
      size = k.get_frame(tx_buffer.data(), tx_buffer.size());
    }

    // Send silence for 1 second
    for (size_t n = 0; n < tx_buffer.size(); ++n)
      tx_buffer[n] = 0.0;
    total_samples = sample_rate;
    while (0 < total_samples && !stoken.stop_requested()) {
      size_t samples_to_send = min(total_samples, tx_buffer.size());
      total_samples -= samples_to_send;
      while (0 < samples_to_send && !stoken.stop_requested()) {
        complex<float> *tx_wrap[1]{tx_buffer.data()};
        samples_to_send -=
            tx_stream.Transmit(tx_wrap, samples_to_send, &tx_meta);
      }
    }

    // Send a sine tone for duration seconds
    total_samples = duration * sample_rate;
    while (0 < total_samples && !stoken.stop_requested()) {
      size_t samples_to_send = min(total_samples, tx_buffer.size());
      total_samples -= samples_to_send;
      for (size_t n = 0; n < samples_to_send; ++n)
        tx_buffer[n] = sinamp * (y *= w);
      while (0 < samples_to_send && !stoken.stop_requested()) {
        complex<float> *tx_wrap[1]{tx_buffer.data()};
        samples_to_send -=
            tx_stream.Transmit(tx_wrap, samples_to_send, &tx_meta);
      }
    }

    // Wait until all samples have been sent
    lime::StreamStats rxStats, txStats;
    tx_stream.StreamStatus(&rxStats, &txStats);
    while (txStats.FIFO.usedCount > 0 && !stoken.stop_requested()) {
      sleep_for(1s * tx_buffer.size() / double(sample_rate));
      tx_stream.StreamStatus(&rxStats, &txStats);
    }

    // Send OFDM frames

    auto t_next = ceil<ten_milliseconds>(sclock.now()) + 50ms;

    for (int n = 0; n < 1s / 10ms * duration && !stoken.stop_requested(); ++n) {
      tx_meta.timestamp = sclock.to_lime(t_next);
      tx_meta.hasTimestamp = true;
      tx_meta.flags = 0;

      fg.assemble(header, reinterpret_cast<unsigned char *>(message.data()),
                  message.size());
      bool last = fg.write(tx_buffer.data(), tx_buffer.size());
      while (not last && !stoken.stop_requested()) {
        complex<float> *tx_wrap[1]{tx_buffer.data()};
        tx_stream.Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
        last = fg.write(tx_buffer.data(), tx_buffer.size());
        tx_meta.hasTimestamp = false;
      }
      tx_meta.flags = StreamTxMeta::EndOfBurst;
      complex<float> *tx_wrap[1]{tx_buffer.data()};
      tx_stream.Transmit(tx_wrap, tx_buffer.size(), &tx_meta);
      t_next += 10ms;
    }
  }

  debug("tx thread stopping");
}

void receive(stop_token stoken, RFStream &rx_stream) {
  debug("rx thread started");

  vector<complex<float>> rx_buffer(1024);
  StreamRxMeta rx_meta;
  grcudp sock("127.0.0.1", 2000);

  while (!stoken.stop_requested()) {
    complex<float> *rx_wrap[1]{rx_buffer.data()};
    rx_stream.Receive(rx_wrap, rx_buffer.size(), &rx_meta);
    // send to udp port, so it can be read with gnuradio
    sock.send(rx_buffer.data(), rx_buffer.size());
  }

  debug("rx thread stopping");
}

int main(int argc, char **argv) {

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  bool is_interactive = true;
  CLI::App app{"hrbeacon: transmit a HAMRAN beacon."};
  argv = app.ensure_utf8(argv);

  OpStatus ops;
  DeviceHandle hint;
  string loglevel = "info";

  // The following parameters can be configured at runtime
  double freq = 53e6;   // Hz
  double txpwr = 11;    // dBm
  double tone = 441;    // Hz
  double duration = 10; // s
  double lna = 30.0;    // dB, RX: Low Noise Amplifier gain
  double tia = 9.0;     // dB, RX: Trans Impedance Amplifier Gain
  double pga = 0.0;     // dB, RX: Programmable Gain Amplifier
  double iamp = 11.0;   // TX: Current Amplifier
  double pad = 47.0;    // dB, TX: Power Amplifier Driver

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
  app.add_option("message", global_message, "beacon message text");

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
        ch.rx.lpf = 2e6;
        ch.rx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.rx.gain[lime::eGainTypes::LNA] = lna; // 0 ... 30
        ch.rx.gain[lime::eGainTypes::TIA] = tia; // 0, 9, 12
        ch.rx.gain[lime::eGainTypes::PGA] = pga; // -12 ... 19
        ch.rx.gfir.enabled = true;
        ch.rx.gfir.bandwidth = 1.86e6;

        ch.tx.enabled = true;
        ch.tx.centerFrequency = freq;
        ch.tx.sampleRate = sample_rate;
        ch.tx.oversample = 1 << 5; // 32
        ch.tx.path = 2;            // TX1_2 0.03 - 1.9 GHz
        ch.tx.lpf = 5.3e6;
        ch.tx.calibrate =
            lime::CalibrationFlag::DCIQ | lime::CalibrationFlag::FILTER;
        ch.tx.gain[lime::eGainTypes::IAMP] = iamp; // 24.0; // 0 ... 63
        ch.tx.gain[lime::eGainTypes::PAD] = pad;   // 44.0;  // 5 ... 55
        ch.tx.gfir.enabled = true;
        ch.tx.gfir.bandwidth = 1.86e6;
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
      if (global_message.empty()) {
        is_interactive = true;
        global_message = "OE1XDU The quick brown fox jumps over the lazy dog.";
      } else {
        info("Beacon text: {}", global_message);
        is_interactive = false;
      }

      jthread rx_thread(receive, ref(*stream.get()));
      jthread tx_thread(transmit, ref(*stream.get()), tone, duration,
                        is_interactive, 5, 2);

      if (is_interactive) {
        cout << "Beacon control thread: interactive mode, enter message or EOF "
                "(Ctrl-D) or empty "
                "line to end."
             << endl;
        message_mutex.lock();
        cout << "Default text is: " << global_message << endl;
        message_mutex.unlock();
        string line;
        cout << "> " << flush;
        while (getline(cin, line)) {
          if (line.empty())
            break;
          message_mutex.lock();
          global_message = line;
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
