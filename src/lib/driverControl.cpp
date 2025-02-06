#include "lib/chassis.h"
#include <cmath>
#include "robotconfig.h"

using namespace lib;

double Chassis::inputCurve(int input, double t) {

  return (exp(-(t/10)) + exp((abs(input) - 127) / 10) * (1 - exp(-(t/10)))) * input;

}

void Chassis::arcadeMod(double forward, double turn, std::vector<double> curves,
                        int speedThreshold) {

  forward = inputCurve(forward, curves[0]);
  turn = inputCurve(turn, curves[1]);

  // arcade drive
  leftMotors->move(forward + turn);
  rightMotors->move(forward - turn);
}

void Chassis::loop(){

  //absolutely nothing

}
