/*
 * SPDX-FileCopyrightText: 2024 Roland Schwarz <roland.schwarz@blackspace.at>
 * SPDX-License-Identifier: GPL-3.0-or-later
*/

// A small test drive for the t3dev class, testing randmomly
// arriving data and its mapping into regular timeslots as required for
// time domain duplexing.

#include <iostream>
using std::cout, std::cerr, std::clog, std::cin, std::endl, std::flush;

#include <fstream>
using std::ofstream, std::ios;

#include <stdexcept>
using std::runtime_error;

#include <exception>
using std::exception;

#include <functional>
using std::ref;

#include <complex>
using std::complex, std::abs, std::exp, std::acos, std::conj;
using namespace std::literals::complex_literals;

#include<vector>
using std::vector;

#include <chrono>
using std::chrono::high_resolution_clock, std::chrono::duration_cast,
  std::chrono::nanoseconds, std::chrono::system_clock;
using namespace std::literals::chrono_literals;
using std::chrono::file_clock;

#include <random>
using std::random_device, std::mt19937, std::uniform_int_distribution;

#include <condition_variable>
using std::condition_variable;

#include <mutex>
using std::mutex, std::unique_lock;

#include <thread>
using std::jthread, std::stop_token, std::this_thread::sleep_for;

#include <csignal>
using std::sig_atomic_t, std::signal;

#include <lime/LimeSuite.h>
#include <lime/Logger.h>

#include <boost/format.hpp>
using boost::format, boost::str;

#include "t3dev.hpp"

namespace {

  volatile sig_atomic_t signal_status = 0;
  void signal_handler(int signal) {
    signal_status = signal;
  }

}

const double sample_rate       = 4e6;
const double band_width        = 2.5e6;
const double slot_length       = 10e-3;
const double carrier_frequency = 53e6;
const double txgain            = 58;
const double rxgain            = 37;


// The receive thread:
void receive(stop_token stoken, t3dev& dev) {
  cout << "Receive thread started" << endl;
  vector<complex<float>> rx_buffer(4'000);
  while(!stoken.stop_requested()) {
      size_t phase; // Can be used in receive situation to synchronize
      // to upstream. Not tested yet. Of course this needs a valid
      // receive signal to be decoded.
      int count = dev.receive(rx_buffer, phase);
      // After decoding the signal and dereiving new_phase
      // dev.set_phase(new_phase)
      // could be used to shift our own transmit sequence to match
      // upstream.
    }
  cout << "Receive thread stopping."  << endl;
}

int main(int argc, char* argv[]) {

  signal(SIGINT, signal_handler); // install handler to catch ctrl-c
  signal(SIGTERM, signal_handler);

  t3dev::init("info");

try {

    t3dev radio(sample_rate, band_width, slot_length, carrier_frequency, txgain, rxgain);

    radio.open();//"LimeSDR Mini, media=USB 3.0, module=FT601, addr=24607:1027, serial=1D90F96D64EBD4");

    vector<complex<float>> tx_buffer(4000);

    jthread rx_thread(receive, ref(radio));

    cout << "Control thread: stop with SIGINT (Ctrl-C) or SIGTERM." << endl;

    // The sine generator with 1kHz frequency.
    complex<double> w = exp(2.0i*acos(-1)*double(1'000.0/sample_rate));
    complex<double> y = conj(w);

    random_device rd;
    mt19937 gen(rd());
    uniform_int_distribution<> distrib(1, 40);

    condition_variable cv;
    mutex cv_m;
    auto next_wakeup = high_resolution_clock::now();

    while(0 == signal_status) {
        unique_lock<mutex> lk(cv_m);
        cv.wait_until(lk, next_wakeup, [] {return false;});
        // Wake up randomly to simulate packets we transmit.
        next_wakeup += distrib(gen)*1ms;

        for (auto& x: tx_buffer) x = (y*=w);
        radio.transmit(tx_buffer);
        for (auto& x: tx_buffer) x = 0.0;
        radio.transmit(tx_buffer);
        for (auto& x: tx_buffer) x = 0.0;
        radio.transmit(tx_buffer);
        for (auto& x: tx_buffer) x = 0.0;
        radio.transmit(tx_buffer);
        for (auto& x: tx_buffer) x = (y*=w);
        radio.transmit(tx_buffer, true);

      }

    cout << "Caught signal " << signal_status << ", exiting ..." << endl;

    rx_thread.request_stop();
    rx_thread.join();

    radio.close();

    cout << "Control thread stopping." << endl;

  } catch(exception& e) {
    cerr << "Exception: " << e.what() << endl;
    return EXIT_FAILURE;
  } catch(...) {
    cerr << "Exception of unknown reason." << endl;
    return EXIT_FAILURE;
  }
    return EXIT_SUCCESS;

}
