/*
 * SPDX-FileCopyrightText: 2024 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

#ifndef _T3DEV_HPP
#define _T3DEV_HPP

#include <vector>
#include <complex>
#include <string>

#include <lime/LimeSuite.h>

// t3dev is a _T_ime _D_omain _D_uplex _D_evice wrapping the lime mini 2.0
// as of writing the initial comment.

//struct lms_device_t; // forward declation of incomplete type

class t3dev {
  lms_device_t*  dev;
  lms_stream_t   rx_stream;
  lms_stream_t   tx_stream;

public:
  const double sample_rate;       // Hz
  const double band_width;        // Hz
  const double carrier_frequency; // Hz
  const double txgain;            // dB
  const double rxgain;            // dB

public:
  // levels: critical, error, warning, info, debug
  static void init(const std::string& level = "info");

public:
  t3dev(double sample_rate, double band_width, double carrier_frequency, double txgain = 34, double rxgain = 37);
  ~t3dev();

  void open(const std::string& id = "");
  void close();

  int receive(std::vector<std::complex<float>>& buf);

  enum class burst {begin, normal, end};
  int transmit(const std::vector<std::complex<float>>& buf, burst phase = burst::normal);

private:
  t3dev(const t3dev&) = delete;

};

#endif // _T3DEV_HPP
