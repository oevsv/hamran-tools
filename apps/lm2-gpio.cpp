//
// SPDX-FileCopyrightText: 2025 Roland Schwarz <roland.schwarz@blackspace.at>
//
// SPDX-License-Identifier: GPL-3.0-or-later
//

#include "gitflow.hpp"
#include <CLI/CLI.hpp>
#include <limesuiteng/limesuiteng.hpp>

#include <rang.hpp>
using rang::style, rang::fgB, rang::fg;

#include <iostream>
using std::cout, std::cerr, std::endl;

#include <exception>
using std::exception;

#include <cstdlib>

int main(int argc, char** argv)
{
    CLI::App app{"Control GPIO lines of lime miniv2", "lm2-gpio"};
    argv = app.ensure_utf8(argv);

    bool verbose = false;
    app.footer("When no options are present the interactive mode is started.");
    app.set_version_flag("--version", gitflow::tag);
    app.add_flag("-v", verbose, "Set verboseness");

    try {
        app.parse(argc, argv);
        if (verbose)
            cout << style::bold << fgB::red << "Hello World!" << fg::reset << style::reset << endl;

    } catch (const CLI::ParseError& e) {
        return app.exit(e);
    } catch (const exception& e) {
        cerr << e.what() << endl;
        return EXIT_FAILURE;
    } catch (...) {
        cerr << "Exception of unknown type";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
