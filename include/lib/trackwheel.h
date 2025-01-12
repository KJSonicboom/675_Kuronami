#pragma once
#include "lib/util.h"
#include "pros/rotation.hpp"

namespace lib {

class TrackingWheel {

private:
  pros::Rotation rot;
  float wheel;

public:
  void reset() { rot.reset_position(); };

  double getDistance() { return rot.get_position() / 36000.0 * wheel * M_PI; }

  TrackingWheel(pros::Rotation rot, float wheel) : rot(rot), wheel(wheel) {
    rot.reset_position();
  }
};
} // namespace lib