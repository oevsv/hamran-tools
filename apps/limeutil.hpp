#ifndef LIME_UTIL_HPP
#define LIME_UTIL_HPP

#include <limesuiteng/RFStream.h>
#include <limesuiteng/limesuiteng.hpp>
#include <spdlog/spdlog.h>

#include <chrono>
#include <cstdint>
#include <string>

namespace lime {
namespace util {

template <std::uintmax_t SAMPLERATE> class stream_clock {
public:
  typedef int64_t rep;
  typedef std::ratio<1l, SAMPLERATE> period;
  typedef std::chrono::duration<rep, period> duration;
  typedef std::chrono::time_point<std::chrono::system_clock,
                                  stream_clock::duration>
      time_point;

  stream_clock(const lime::RFStream &stream)
      : stream(stream), epoch(std::chrono::time_point_cast<duration>(
                                  std::chrono::system_clock::now()) -
                              duration(stream.GetHardwareTimestamp())) {}

  time_point now() noexcept {
    return duration(stream.GetHardwareTimestamp()) + epoch;
  }

  static constexpr bool is_steady = false;

private:
  const lime::RFStream &stream;
  time_point epoch;
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
