/*
 * SPDX-FileCopyrightText: 2024 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

// t3dev is a _T_ime _D_omain _D_uplex _D_evice wrapping the lime mini 2.0
// as of writing the initial comment.

#ifndef _T3DEV_HPP
#define _T3DEV_HPP

#include <cstddef>
#include <vector>
#include <complex>
#include <string>
#include <atomic>
#include <chrono>
#include <lime/LimeSuite.h>

class t3dev {
  lms_device_t*  dev;

  lms_stream_t      rx_stream;
  lms_stream_meta_t rx_meta;

  lms_stream_t      tx_stream;
  lms_stream_meta_t tx_meta;

  std::size_t              current_phase;
  std::atomic<std::size_t> new_phase;

public:
  const double      sample_rate;       // Hz
  const double      band_width;        // Hz
  const double      slot_length;       // s, remainder of slot is for receive
  const std::size_t max_phase;         // Samples, slot length in samples
  const double      carrier_frequency; // Hz
  const double      txgain;            // dB
  const double      rxgain;            // dB

public:
  // levels: critical, error, warning, info, debug
  static void init(const std::string& level = "info");

public:
  t3dev(double sample_rate, double band_width, double slot_length, double carrier_frequency, double txgain = 34, double rxgain = 37);
  ~t3dev();

  void open(const std::string& id = "");
  void close();

  void set_phase(std::size_t phase);

  int receive(std::vector<std::complex<float>>& buf);
  int receive(std::vector<std::complex<float>>& buf, std::size_t& phase);

  int transmit(const std::vector<std::complex<float>>& buf, bool end_of_slot = false);

private:
  t3dev(const t3dev&) = delete;

};

#endif // _T3DEV_HPP
