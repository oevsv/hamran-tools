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
using std::cout, std::endl;

int main()
{
    cout << style::bold << fgB::red << "Hello World!" << fg::reset << style::reset << endl;
    cout << gitflow::tag << endl;
    return 0;
}
