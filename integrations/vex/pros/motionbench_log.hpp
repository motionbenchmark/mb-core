// motionbench_log.hpp — onboard pose logger for VEX V5 (PROS 4).
//
// Writes a MotionBench `poses.csv` to the SD card at a fixed rate from any
// odometry source (LemLib, EZ-Template, your own). Copy the file from the SD
// card into a `.mbtrace` directory next to a manifest.json, or use:
//   python -c "import motionbench as mb; ..."  (see README in this folder)
//
// Usage:
//   #include "motionbench_log.hpp"
//   // pose in inches / degrees, as most V5 libraries report it
//   motionbench::Logger mblog([] {
//     auto p = chassis.getPose();               // LemLib example
//     return motionbench::PoseInDeg{p.x, p.y, p.theta};
//   });
//   void autonomous() { mblog.start("/usd/mb_auton.csv"); ... ; mblog.stop(); }
//
// Field frame: MotionBench uses metres with the origin at the near-left field
// corner and heading counter-clockwise from +x. Set `origin_offset_x_in`, `origin_offset_y_in` and
// `heading_convention` to map your library's frame onto it.
//
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <functional>

#include "pros/rtos.hpp"

namespace motionbench {

struct PoseInDeg {
  double x_in;
  double y_in;
  double theta_deg;
};

enum class Heading {
  CcwFromX,  // math convention: 0 = +x, counter-clockwise positive (MotionBench native)
  CwFromY,   // compass convention used by many V5 libraries: 0 = +y, clockwise positive
};

class Logger {
 public:
  explicit Logger(std::function<PoseInDeg()> source, int period_ms = 20,
                  Heading heading_convention = Heading::CwFromY,
                  double origin_offset_x_in = 0.0, double origin_offset_y_in = 0.0)
      : source_(std::move(source)),
        period_ms_(period_ms),
        heading_(heading_convention),
        ox_(origin_offset_x_in),
        oy_(origin_offset_y_in) {}

  ~Logger() { stop(); }

  // Start logging to `path` (must be on the SD card, i.e. start with "/usd/").
  bool start(const char* path) {
    stop();
    file_ = std::fopen(path, "w");
    if (!file_) return false;
    std::fprintf(file_, "t_s,x_m,y_m,theta_rad\n");
    running_ = true;
    t0_ = pros::millis();
    task_ = new pros::Task([this] { loop(); }, "motionbench-log");
    return true;
  }

  void stop() {
    if (!running_) return;
    running_ = false;
    pros::delay(period_ms_ * 2);
    if (file_) {
      std::fflush(file_);
      std::fclose(file_);
      file_ = nullptr;
    }
    delete task_;  // the RTOS task has already left its loop; this frees the handle
    task_ = nullptr;
  }

 private:
  static constexpr double kInToM = 0.0254;
  static constexpr double kPi = 3.14159265358979323846;

  void loop() {
    std::uint32_t wake = pros::millis();
    int since_flush = 0;
    while (running_) {
      PoseInDeg p = source_();
      double th = p.theta_deg * kPi / 180.0;
      if (heading_ == Heading::CwFromY) th = kPi / 2.0 - th;
      th = std::atan2(std::sin(th), std::cos(th));
      double t = (pros::millis() - t0_) / 1000.0;
      if (file_) {
        std::fprintf(file_, "%.3f,%.4f,%.4f,%.4f\n", t, (p.x_in + ox_) * kInToM,
                     (p.y_in + oy_) * kInToM, th);
        if (++since_flush >= 25) {  // flush about twice a second at 50 Hz
          std::fflush(file_);
          since_flush = 0;
        }
      }
      pros::Task::delay_until(&wake, period_ms_);
    }
  }

  std::function<PoseInDeg()> source_;
  int period_ms_;
  Heading heading_;
  double ox_, oy_;
  std::FILE* file_ = nullptr;
  pros::Task* task_ = nullptr;
  volatile bool running_ = false;
  std::uint32_t t0_ = 0;
};

}  // namespace motionbench
