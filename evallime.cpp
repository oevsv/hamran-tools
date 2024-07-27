/*
 * SPDX-FileCopyrightText: 2024 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl, std::flush;

#include <fstream>
using std::ofstream, std::ios;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <complex>
using std::complex, std::abs;
using namespace std::literals::complex_literals;

#include<vector>
using std::vector;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::duration_cast,
  std::chrono::nanoseconds, std::chrono::system_clock;
using namespace std::literals::chrono_literals;
using std::chrono::file_clock;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <boost/format.hpp>
using boost::format, boost::str;

#include "t3dev.hpp"

namespace {

  const lime::LogLevel default_LogLevel = lime::LOG_LEVEL_INFO;
  lime::LogLevel max_LogLevel = default_LogLevel;

  void limeSuiteLogHandler(const lime::LogLevel level, const char* message) {
    if (level <= max_LogLevel) {
        switch(level) {
          //rem: throwing on error avoids overly verbose error handling code
          case lime::LOG_LEVEL_CRITICAL: throw runtime_error(message);
          case lime::LOG_LEVEL_ERROR:    throw runtime_error(message);
          case lime::LOG_LEVEL_WARNING: clog << format("Warning: %s\n") % message; return;
          case lime::LOG_LEVEL_INFO:    clog << format("Info: %s\n") % message; return;
          case lime::LOG_LEVEL_DEBUG:   clog << format("Debug: %s\n") % message; return;
          }
      }
  }

  volatile sig_atomic_t signal_status = 0;
  void signal_handler(int signal) {
    signal_status = signal;
  }

}

const double sample_rate = 4e6;
const double freq        = 53e6;
const double txpwr       = 0;

int main(int argc, char* argv[]) {

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c
  signal(SIGTERM, signal_handler);

  max_LogLevel = lime::LOG_LEVEL_INFO;
  lime::registerLogHandler(&limeSuiteLogHandler);

  lms_device_t* dev = nullptr;

  t3dev::init("info");

try {

    t3dev lime2(sample_rate, 2.5e6, freq, 34, 37);

    lime2.open();//"LimeSDR Mini, media=USB 3.0, module=FT601, addr=24607:1027, serial=1D90F96D64EBD4");

    lime2.close();

    return EXIT_SUCCESS;

    // Try to open devices until one is available or fail if none.
    lms_info_str_t list[8];
    int numdev = LMS_GetDeviceList(list);
    if (numdev < 1) lime::error("No device found");
    int n = 0;
    for (; n < numdev; ++n) {
        try {
          LMS_Open(&dev, list[n], nullptr);
          lime::info("Device: %s", list[n]);
          break;
        } catch(runtime_error& e) { /* possibly busy, try next one */}
      }
    if (n == numdev)
      lime::error("No device could be obened");

    LMS_Init(dev);

    LMS_SetSampleRate(dev,     sample_rate, 0);

    // initialize receive direction
    LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
    LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
    LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, freq);
    LMS_SetNormalizedGain(dev, LMS_CH_RX, 0, 1.0);
    LMS_SetGFIRLPF(dev,        LMS_CH_RX, 0, true, 1.86e6);
    LMS_SetLPFBW(dev,          LMS_CH_RX, 0, 5e6);

    lms_stream_t rx_stream;
    rx_stream.channel = 0;
    rx_stream.fifoSize = 1024*1024;
    rx_stream.throughputVsLatency = 0.5;
    rx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    rx_stream.isTx = false;

    // initialize transmit direction
    LMS_EnableChannel(dev,     LMS_CH_TX, 0, true);
    LMS_SetLOFrequency(dev,    LMS_CH_TX, 0, freq);
    LMS_SetAntenna(dev,        LMS_CH_TX, 0, LMS_PATH_TX2);
    LMS_SetGaindB(dev,         LMS_CH_TX, 0, 58+txpwr);
    LMS_SetGFIRLPF(dev,        LMS_CH_TX, 0, true, 1.86e6);
    LMS_SetLPFBW(dev,          LMS_CH_TX, 0, 5e6);

    lms_stream_t tx_stream;
    tx_stream.channel = 0;
    tx_stream.fifoSize = 1024*1024;
    tx_stream.throughputVsLatency = 0.5;
    assert(8*sizeof(float) == 32);
    tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
    tx_stream.isTx = true;

    LMS_Calibrate(dev,         LMS_CH_RX, 0, 2.5e6, 0);
    LMS_Calibrate(dev,         LMS_CH_TX, 0, 2.5e6, 0);

    LMS_SetupStream(dev, &rx_stream);
    LMS_SetupStream(dev, &tx_stream);

    LMS_StartStream(&rx_stream);
    LMS_StartStream(&tx_stream);

    uint64_t sample_clock;

    vector<complex<float>> tx_buffer(4000);
    lms_stream_meta_t tx_meta;
    vector<high_resolution_clock::duration> tx_delta;
    vector<high_resolution_clock::duration> tx_clock;
    vector<int> tx_count;

    vector<complex<float>> rx_buffer(4000);
    lms_stream_meta_t rx_meta;
    vector<uint64_t> rx_timestamp;
    vector<high_resolution_clock::duration> rx_delta;
    vector<high_resolution_clock::duration> rx_clock;
    vector<int> rx_count;

    high_resolution_clock::time_point start, stop, meas_start;

    cout << "Control thread: stop with SIGINT (Ctrl-C) or SIGTERM." << endl;
    //while (0 == signal_status) {
    tx_meta.waitForTimestamp = false;
    meas_start = high_resolution_clock::now();
    for (size_t n=0; n< 10'000; ++n) {

        start = high_resolution_clock::now();
        int tx_ret = LMS_SendStream(&tx_stream, tx_buffer.data(), tx_buffer.size(), &tx_meta, 1000);
        stop = high_resolution_clock::now();
        tx_delta.push_back(stop-start);
        tx_clock.push_back(stop-meas_start);
        tx_count.push_back(tx_ret);

        start = high_resolution_clock::now();
        int rx_ret = LMS_RecvStream(&rx_stream, rx_buffer.data(), rx_buffer.size(), &rx_meta, 1000);
        stop = high_resolution_clock::now();
        sample_clock = rx_meta.timestamp;
        rx_delta.push_back(stop-start);
        rx_timestamp.push_back(rx_meta.timestamp);
        rx_clock.push_back(start-meas_start);
        rx_count.push_back(rx_ret);
      }
    cout << "Caught signal " << signal_status << ", exiting ..." << endl;

    LMS_StopStream(&tx_stream);
    LMS_DestroyStream(dev, &tx_stream);
    LMS_EnableChannel(dev, LMS_CH_TX, 0, false);

    LMS_StopStream(&rx_stream);
    LMS_DestroyStream(dev, &rx_stream);
    LMS_EnableChannel(dev, LMS_CH_RX, 0, false);

    LMS_Close(dev);
    dev = nullptr;

//    ofstream out("timestamps.txt", ios::out);
//    for (size_t n=0; n<rx_timestamp.size(); ++n) {
//        out << format("%016d %d %016d %016d %d %016d %016d\n")
//               % rx_timestamp[n]
//               % rx_count[n]
//               % duration_cast<nanoseconds>(rx_delta[n]).count()
//               % duration_cast<nanoseconds>(rx_clock[n]).count()
//               % tx_count[n]
//               % duration_cast<nanoseconds>(tx_delta[n]).count()
//               % duration_cast<nanoseconds>(tx_clock[n]).count();
//      }
//    out.close();

    cout << "Control thread stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    if (nullptr != dev)
      LMS_Close(dev);
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    if (nullptr != dev)
      LMS_Close(dev);
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;

}
