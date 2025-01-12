#pragma once
#include <cmath> // math functions like sqrt()
// provides information about the properties of arithmetic types
#include <limits>

class Point {
public:
  // values at any point
  float x;
  float y;
  /* theta is initialized at NaN so theta does not
   need to be inputted */
  float theta = std::numeric_limits<float>::quiet_NaN();

  // Constructor
  Point(float x, float y, float theta = std::numeric_limits<float>::quiet_NaN()): x(x), y(y), theta(theta) {}

  // Method to calculate distance between two points
  float distanceTo(const Point &other) const {
    float deltaX = x - other.x;
    float deltaY = y - other.y;
    return sqrt(deltaX * deltaX + deltaY * deltaY);
  }
  // Method to convert degrees to radians  float
  float degreesToRadians(float degrees) { return degrees * M_PI / 180.0; }

  // angular error
  float angleError(const Point &other) { return other.theta - theta; }

  float angleTo(const Point &other) {
    float angle = atan2(other.y - y, other.x - x);
    return angle - theta;
  }

  Point operator-(const Point &other) const {
    return Point(this->x - other.x, this->y - other.y, this->theta);
  }
  Point operator+(const Point &other) const {
    return Point(this->x + other.x, this->y + other.y, this->theta);
  }
  Point operator*(const float &other) const {
    return Point(this->x * other, this->y * other, this->theta);
  }
};