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

#include <iostream>
using std::cin, std::cout, std::cerr, std::endl, std::flush;

#include <string>
using std::string, std::getline;

#include <stdexcept>
using std::exception;

int main(int argc, char **argv) {

  int main_exit = EXIT_SUCCESS;
  string loglevel = "info";

  try {
    spdlog::set_level(spdlog::level::from_str(loglevel));
    spdlog::flush_on(spdlog::get_level());

    tuntap::tuntap tun(TUNTAP_MODE_TUNNEL);

    cout << "interactive mode, enter message or EOF "
            "(Ctrl-D) or empty "
         << endl;
    string line;
    while (getline(cin, line)) {
      if (line.empty())
        break;
      cout << line << endl;
      cout << "> " << flush;
    }

  } catch (const exception &e) {
    critical("{}", e.what());
    main_exit = EXIT_FAILURE;
  } catch (...) {
    critical("Exception of unknown type");
    main_exit = EXIT_FAILURE;
  }

  return main_exit;
}
