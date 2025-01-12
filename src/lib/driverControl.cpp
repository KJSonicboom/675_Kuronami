#include "lib/chassis.h"
#include <cmath>
#include "robotconfig.h"

using namespace lib;

double Chassis::inputCurve(int input, double t) {

  double output = (exp(-(t/10)) + exp((abs(input) - 127) / 10) * (1 - exp(-(t/10)))) * input;

  return output;

}

void Chassis::arcade(double forward, double turn, std::vector<double> curves) {
  // curve input
  forward = inputCurve(forward, curves[0]);
  turn = inputCurve(turn, curves[1]);
}
void Chassis::arcade(double forward, double turn){
  // arcade drive
  leftMotors->move(forward + turn);
  rightMotors->move(forward - turn);
}

void Chassis::tank(double left, double right, std::vector<double> curves) {
  // curve input
  left = inputCurve(left, curves[0]);
  right = inputCurve(right, curves[1]);

  // tank drive
  leftMotors->move(left);
  rightMotors->move(right);
}

void Chassis::arcadeMod(double forward, double turn, std::vector<double> curves,
                        int speedThreshold) {

  forward = inputCurve(forward, curves[0]);
  turn = inputCurve(turn, curves[1]);

  std::string col = std::to_string(forward);
  controller.set_text(0, 5, col);

  // arcade drive
  leftMotors->move(forward + turn);
  rightMotors->move(forward - turn);
}
