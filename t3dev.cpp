/*
 * SPDX-FileCopyrightText: 2024 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#include "t3dev.hpp"

using std::vector;
using std::complex;
using std::string;
using std::size_t;
using namespace std::literals::chrono_literals;

#include <iostream>
using std::clog;

#include <stdexcept>
using std::runtime_error;

#include <algorithm>
using std::clamp;

#include <cstdlib>
using std::div;

#include <boost/format.hpp>
using boost::format;

#include <lime/LimeSuite.h>
#include <lime/Logger.h>


namespace {

  struct { const char* key; lime::LogLevel val; } levelmap[] = {
  {"critical", lime::LOG_LEVEL_CRITICAL},
  {"error",    lime::LOG_LEVEL_ERROR},
  {"warning",  lime::LOG_LEVEL_WARNING},
  {"info",     lime::LOG_LEVEL_INFO},
  {"debug",    lime::LOG_LEVEL_DEBUG}
  };

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

}

// One time initialization of lime library
void t3dev::init(const string& level) {
  for (size_t n = 0; n<5; ++n)
    if (level == levelmap[n].key) max_LogLevel = levelmap[n].val;
  lime::registerLogHandler(&limeSuiteLogHandler);
}

t3dev::t3dev(double sample_rate, double band_width, double slot_length, double carrier_frequency, double txgain, double rxgain) :
  dev(nullptr),  current_phase(0), new_phase(0),
  sample_rate(sample_rate), band_width(band_width),
  slot_length(slot_length), max_phase(size_t(sample_rate*slot_length)),
  carrier_frequency(carrier_frequency),
  txgain(txgain), rxgain(rxgain)
   {

}

t3dev::~t3dev() {
  // Only last ressort, better close before calling destructor.
  try { close(); } catch (...) {};
}

// Open device specified by id or list all devices
// and open first available if id is empty.
void t3dev::open(const string& id) {

  if (id.empty()) {
      int numdev = LMS_GetDeviceList(nullptr);
      if (0 != numdev) {
          lms_info_str_t list[numdev];
          numdev = LMS_GetDeviceList(list);
          int n = 0;
          lime::info("Devices found:");
          for (; n<numdev; ++n) {
              lime::info(list[n]);
            }
          n = 0;
          lime::info("Trying to open first available:");
          for(; n<numdev; ++n) {
              try {
                LMS_Open(&dev, list[n], nullptr);
                lime::info("Opened: %s", list[n]);
                break;
              } catch(runtime_error& e) { /* possibly busy, try next one */}
            }
          if (n == numdev)
            lime::error("No device could be opened");
        }
      else lime::error("no device found");
    }
  else {
      LMS_Open(&dev, id.c_str(), nullptr);
      lime::info("Opened: %s", id.c_str());
    }

  // Initialize device and prepare streams
  LMS_Init(dev);

  // Set up GPIO.
  uint8_t gpio_dir = 0xFF;
  LMS_GPIODirWrite(dev, &gpio_dir, 1);

  uint8_t gpio_val = 0;
  LMS_GPIODirRead(dev, &gpio_val, 1);
  lime::debug("GPIODIR: 0x%02x", unsigned(gpio_val));

  // Set up TX indicator on GPIO0.
  // https://github.com/myriadrf/LimeSDR-Mini-v2_GW/issues/3
  // https://discourse.myriadrf.org/t/limemini-2-2-gpio/8012/5
  // See also HW description https://github.com/myriadrf/LimeSDR-Mini-v2
  uint16_t fpga_val = 0;
  LMS_ReadFPGAReg(dev, 0x00c0, &fpga_val);
  fpga_val &= 0xfffe;
  LMS_WriteFPGAReg(dev, 0x00c0, fpga_val);

  // The maximum fpga0 can be switched on before the signal and
  // off after the signal is 38 us. This corresponds to
  // TX_ANT_PRE = 0 and TX_ANT_POST = 0.

  // Set TXANT_PRE
  const double on_after_start = 500e-6; // >= -38e-6;
  fpga_val = static_cast<uint16_t>((38e-6 + on_after_start)*sample_rate);
  LMS_WriteFPGAReg(dev, 0x0010, fpga_val);

  // Set TXANT_POST
  const double off_after_end = 500e-6; // >= -38e-6;
  fpga_val = static_cast<uint16_t>((38e-6 + off_after_end)*sample_rate);
  LMS_WriteFPGAReg(dev, 0x0011, fpga_val);

  // Set the sample rate
  LMS_SetSampleRate(dev, sample_rate, 0);
  double host_sample_rate, rf_sample_rate;

  // initialize receive direction
  LMS_GetSampleRate(dev, LMS_CH_RX, 0, &host_sample_rate, &rf_sample_rate);
  lime::info("Rx sample rate host %.3f MHz, rf %.3f MHz", host_sample_rate*1e-6, rf_sample_rate*1e-6);
  LMS_EnableChannel(dev,     LMS_CH_RX, 0, true);
  LMS_SetAntenna(dev,        LMS_CH_RX, 0, LMS_PATH_LNAW);
  LMS_SetLOFrequency(dev,    LMS_CH_RX, 0, carrier_frequency);
  LMS_SetGaindB(dev,         LMS_CH_RX, 0, rxgain);
  unsigned actual_rxgain;
  LMS_GetGaindB(dev,         LMS_CH_RX, 0, &actual_rxgain);
  lime::info("rx gain: %d", actual_rxgain);
  lms_range_t rxbw_limits;
  LMS_GetLPFBWRange(dev,     LMS_CH_RX, &rxbw_limits);
  double analog_rxbw = clamp(band_width, rxbw_limits.min, rxbw_limits.max);
  LMS_SetLPFBW(dev,          LMS_CH_RX, 0, analog_rxbw);
  LMS_GetLPFBW(dev,          LMS_CH_RX, 0, &analog_rxbw);
  lime::info("Rx LPFBW %.3f MHz", analog_rxbw*1e-6);
  if (0 == LMS_SetGFIRLPF(dev,        LMS_CH_RX, 0, true, band_width)) {
      lime::info("Rx GFIRLPF  %.3f MHz", band_width*1e-6);
    }
  LMS_Calibrate(dev,         LMS_CH_RX, 0, band_width, 0);

  rx_stream.channel = 0;
  rx_stream.fifoSize = 1024*1024;
  rx_stream.throughputVsLatency = 0.5;
  rx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
  rx_stream.isTx = false;
  LMS_SetupStream(dev, &rx_stream);

  // initialize transmit direction
  LMS_GetSampleRate(dev, LMS_CH_TX, 0, &host_sample_rate, &rf_sample_rate);
  lime::info("Tx sample rate host %.3f MHz, rf %.3f MHz", host_sample_rate*1e-6, rf_sample_rate*1e-6);
  LMS_EnableChannel(dev,     LMS_CH_TX, 0, true);
  LMS_SetAntenna(dev,        LMS_CH_TX, 0, LMS_PATH_TX2);
  LMS_SetLOFrequency(dev,    LMS_CH_TX, 0, carrier_frequency);
  LMS_SetGaindB(dev,         LMS_CH_TX, 0, txgain);
  unsigned actual_txgain;
  LMS_GetGaindB(dev,         LMS_CH_TX, 0, &actual_txgain);
  lime::info("tx gain: %d", actual_txgain);
  lms_range_t txbw_limits;
  LMS_GetLPFBWRange(dev,     LMS_CH_TX, &txbw_limits);
  double analog_txbw = clamp(band_width, txbw_limits.min, txbw_limits.max);
  LMS_SetLPFBW(dev,          LMS_CH_TX, 0, analog_txbw);
  LMS_GetLPFBW(dev,          LMS_CH_TX, 0, &analog_txbw);
  lime::info("Tx LPFBW %.3f MHz", analog_txbw*1e-6);
  if (0 == LMS_SetGFIRLPF(dev,        LMS_CH_TX, 0, true, band_width)) {
      lime::info("Tx GFIRLPF  %.3f MHz", band_width*1e-6);
    }
  LMS_Calibrate(dev,         LMS_CH_TX, 0, band_width, 0);

  tx_stream.channel = 0;
  tx_stream.fifoSize = 1024*1024;
  tx_stream.throughputVsLatency = 0.5;
  assert(8*sizeof(float) == 32);
  tx_stream.dataFmt = lms_stream_t::LMS_FMT_F32;
  tx_stream.isTx = true;
  LMS_SetupStream(dev, &tx_stream);

  LMS_StartStream(&rx_stream);
  LMS_StartStream(&tx_stream);

  rx_meta.timestamp = 0;

  tx_meta.waitForTimestamp = true;
  tx_meta.flushPartialPacket = false;
  tx_meta.timestamp = 0;

  lime::info("Slot length in samples %d", max_phase);
}

void t3dev::close() {
  if (nullptr != dev) {
      lime::info("Stopping streams and closing device.");
      LMS_StopStream(&tx_stream);
      LMS_DestroyStream(dev, &tx_stream);
      LMS_EnableChannel(dev, LMS_CH_TX, 0, false);
      LMS_StopStream(&rx_stream);
      LMS_DestroyStream(dev, &rx_stream);
      LMS_EnableChannel(dev, LMS_CH_RX, 0, false);
      LMS_Close(dev);
      dev = nullptr;
    }
}

int t3dev::receive(vector<complex<float>>& buf) {
  return LMS_RecvStream(&rx_stream, buf.data(), buf.size(), &rx_meta, 1000);
}

int t3dev::receive(vector<complex<float>>& buf, size_t& phase) {
  int count = LMS_RecvStream(&rx_stream, buf.data(), buf.size(), &rx_meta, 1000);
  phase = rx_meta.timestamp % max_phase; // phase of first sample
  return count;
}

// Set duplex phase from different thread. Not tested yet, OE1RSA.
void t3dev::set_phase(size_t phase) {
  new_phase.store(phase+1); // zero used as flag, so move away from zero
}

int t3dev::transmit(const vector<complex<float>>& buf, bool end_of_slot) {

  if (tx_meta.waitForTimestamp) {
      lms_stream_status_t status;
      LMS_GetStreamStatus(&rx_stream, &status);
      uint64_t next_timestamp = ((status.timestamp+current_phase)/max_phase + 2)*max_phase;
      if (next_timestamp > tx_meta.timestamp)
        tx_meta.timestamp = next_timestamp;
      else
        tx_meta.timestamp += max_phase;
      size_t adjust_phase = new_phase.exchange(0);
      if (adjust_phase) {
          adjust_phase = (adjust_phase-1) % max_phase;
          tx_meta.timestamp += (max_phase-current_phase + adjust_phase) % max_phase;
          current_phase = adjust_phase;
        }
    }
  tx_meta.flushPartialPacket = end_of_slot;
  int count = LMS_SendStream(&tx_stream, buf.data(), buf.size(), &tx_meta, 1000);
  tx_meta.waitForTimestamp = end_of_slot;

  return count;
}
