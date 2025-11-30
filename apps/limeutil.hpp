#ifndef LIME_UTIL_HPP
#define LIME_UTIL_HPP

#include <limesuiteng/RFStream.h>
#include <limesuiteng/Timespec.h>
#include <limesuiteng/limesuiteng.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace lime {
namespace util {

template <std::uintmax_t SAMPLERATE> class stream_clock;

template <std::uintmax_t SAMPLERATE, class Duration>
using stream_time = std::chrono::time_point<stream_clock<SAMPLERATE>, Duration>;

template <std::uintmax_t SAMPLERATE> class stream_clock {
public:
  typedef int64_t rep;
  typedef std::ratio<1l, SAMPLERATE> period;
  typedef std::chrono::duration<rep, period> duration;
  typedef std::chrono::time_point<stream_clock> time_point;
  static constexpr bool is_steady = false;

  // stream_clock(const lime::RFStream &stream)
  //     : stream(stream), sys_epoch(std::chrono::time_point_cast<duration>(
  //                                     std::chrono::system_clock::now()) -
  //                                 duration(stream.GetHardwareTimestamp())) {}

  stream_clock(const lime::RFStream &stream)
      : stream(stream), sys_epoch(std::chrono::system_clock::now() -
                                  duration(stream.GetHardwareTimestamp())) {}

  time_point now() noexcept {
    return duration(stream.GetHardwareTimestamp()) + time_point();
  }

  template <class Duration>
  static auto from_sys(std::chrono::sys_time<Duration> const &tp) noexcept {
    return stream_time(tp);
  }

  template <class Duration>
  static auto to_sys(stream_time<SAMPLERATE, Duration> const &tp) noexcept {
    return std::chrono::system_clock::time_point(); // + tp;
  }

private:
  const lime::RFStream &stream;
  // time_point sys_epoch;
  std::chrono::system_clock::time_point sys_epoch;
};

void limeSuiteLogHandler(const lime::LogLevel level,
                         const std::string &message) {
  switch (level) {
  case lime::LogLevel::Critical:
    spdlog::critical("lime: {}", message);
    break;
  case lime::LogLevel::Error:
    spdlog::error("lime: {}", message);
    break;
  case lime::LogLevel::Warning:
    spdlog::warn("lime: {}", message);
    break;
  case lime::LogLevel::Info:
    spdlog::info("lime: {}", message);
    break;
  case lime::LogLevel::Debug:
    spdlog::debug("lime: {}", message);
    break;
  case lime::LogLevel::Verbose:
    spdlog::trace("lime: {}", message);
    break;
  }
}

} // namespace util
} // namespace lime

template <std::uintmax_t SAMPLERATE>
struct std::chrono::is_clock<lime::util::stream_clock<SAMPLERATE>> : true_type {
};
template <std::uintmax_t SAMPLERATE>
inline constexpr bool
    std::chrono::is_clock_v<lime::util::stream_clock<SAMPLERATE>> = true;

#endif // LIME_UTIL_HPP
