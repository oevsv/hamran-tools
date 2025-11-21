//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "gitflow.hpp"

#include <tuntap++.hh>

#include <spdlog/spdlog.h>
using spdlog::trace, spdlog::debug, spdlog::info, spdlog::warn, spdlog::error,
    spdlog::critical;

#include <cstdlib>

#include <cstddef>
using std::byte, std::to_integer;

#include <iostream>
using std::cin, std::cout, std::cerr, std::endl, std::flush;

#include <string>
using std::string, std::getline;

#include <vector>
using std::vector;

#include <exception>
using std::exception;

#include <csignal>
using std::sig_atomic_t, std::signal;

namespace {

volatile sig_atomic_t signal_status = 0;
void signal_handler(int signal) { signal_status = signal; }

extern "C" void tuntapLogHandler(int level, const char *errmsg) {

  switch (level) {
  case TUNTAP_LOG_ERR:
    spdlog::critical("libtuntap error: {}", errmsg);
    break;
  case TUNTAP_LOG_WARN:
    spdlog::error("libtuntap warn: {}", errmsg);
    break;
  case TUNTAP_LOG_NOTICE:
    spdlog::warn("libtuntap notice: {}", errmsg);
    break;
  case TUNTAP_LOG_INFO:
    spdlog::info("libtuntap: {}", errmsg);
    break;
  case TUNTAP_LOG_DEBUG:
    spdlog::debug("libtuntap: {}", errmsg);
    break;
  case TUNTAP_LOG_NONE:
  default:
    spdlog::trace("libtuntap: {}", errmsg);
    break;
  }
}

} // namespace

int main(int argc, char **argv) {

  int main_exit = EXIT_SUCCESS;

  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  string loglevel = "info";

  try {
    spdlog::set_level(spdlog::level::from_str(loglevel));
    spdlog::flush_on(spdlog::get_level());

    tuntap::tuntap tun(TUNTAP_MODE_TUNNEL);
    tuntap_log_set_cb(tuntapLogHandler);

    info("name: {}", tun.name());
    info("mtu: {}", tun.mtu());
    tun.ip("10.23.3.0", 24);

    tun.up();

    vector<byte> rx_buf(tun.mtu());

    while (0 == signal_status) {
      int rc = tun.read(rx_buf.data(), rx_buf.size(), 1000);
      if (-1 < rc) {
        cout << "Size: " << rc << ", ";
        int vers = to_integer<unsigned>(rx_buf[0] >> 4);
        if (4 == vers) {
          cout << "VERS: " << 4 << ", ";
          cout << "HLEN: " << to_integer<int>(byte{0x0f} & rx_buf[0]) << ", ";
          cout << "SRC: " << to_integer<unsigned>(rx_buf[12]) << "."
               << to_integer<unsigned>(rx_buf[13]) << "."
               << to_integer<unsigned>(rx_buf[14]) << "."
               << to_integer<unsigned>(rx_buf[15]) << ", ";
          cout << "DST: " << to_integer<unsigned>(rx_buf[16]) << "."
               << to_integer<unsigned>(rx_buf[17]) << "."
               << to_integer<unsigned>(rx_buf[18]) << "."
               << to_integer<unsigned>(rx_buf[19]) << ", ";
        } else if (6 == vers) {
          cout << "VERS: " << 6 << ", ";
        } else {
          cout << "VERS: " << vers << endl;
        }
        cout << "end" << endl;
      }
    }

    tun.down();

  } catch (const exception &e) {
    critical("{}", e.what());
    main_exit = EXIT_FAILURE;
  } catch (...) {
    critical("Exception of unknown type");
    main_exit = EXIT_FAILURE;
  }

  return main_exit;
}
