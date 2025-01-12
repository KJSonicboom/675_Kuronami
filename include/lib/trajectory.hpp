#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include "lib/point.hpp"

struct ProfilePoint {
    Point position;
    double distance = 0;
    double velocity = 0;
    double leftVelocity = 0;
    double rightVelocity = 0;
    double curvature = 0;

    ProfilePoint() : position(0, 0) {}
};

class MotionProfileGenerator {
private:
    static constexpr double DISTANCE_STEP = 0.125;

    Point evaluateBezier(const std::vector<Point>& controls, double t) {
        std::vector<Point> points = controls;
        int n = points.size();

        for (int i = 1; i < n; i++) {
            for (int j = 0; j < n - i; j++) {
                points[j] = points[j] * (1 - t) + points[j + 1] * t;
            }
        }
        return points[0];
    }

    Point evaluateBezierDerivative(const std::vector<Point>& controls, double t) {
        std::vector<Point> derivatives;
        int n = controls.size() - 1;

        for (int i = 0; i < n; i++) {
            derivatives.push_back((controls[i + 1] - controls[i]) * n);
        }
        return evaluateBezier(derivatives, t);
    }

    Point evaluateBezierSecondDerivative(const std::vector<Point>& controls, double t) {
        std::vector<Point> derivatives;
        int n = controls.size() - 1;

        for (int i = 0; i < n - 1; i++) {
            derivatives.push_back((controls[i + 2] - controls[i + 1] * 2 + controls[i]) * (n * (n - 1)));
        }

        return evaluateBezier(derivatives, t);
    }

    double calculateSignedCurvature(const std::vector<Point>& controls, double t) {
        Point d1 = evaluateBezierDerivative(controls, t);
        Point d2 = evaluateBezierSecondDerivative(controls, t);

        // Cross product of first derivative and second derivative
        double numerator = d1.x * d2.y - d1.y * d2.x;
        double denominator = std::pow(d1.x * d1.x + d1.y * d1.y, 1.5);

        // Prevent division by zero
        if (std::abs(denominator) < 1e-6) {
            return 0.0;
        }

        return numerator / denominator;
    }

public:
    std::vector<ProfilePoint> generateProfile(
        const std::vector<Point>& controlPoints,
        double maxVelocity,
        double maxAccel,
        double maxDecel,
        double trackWidth
    ) {
        std::vector<ProfilePoint> profile;
        double totalDistance = 0;
        double lastT = 0;

        // Generate initial path points
        for (double t = 0; t <= 1.0; t += 0.001) {
            Point current = evaluateBezier(controlPoints, t);
            if (t > 0) {
                Point prev = evaluateBezier(controlPoints, lastT);
                double segmentDist = std::sqrt(
                    std::pow(current.x - prev.x, 2) +
                    std::pow(current.y - prev.y, 2)
                );
                totalDistance += segmentDist;
            }

            ProfilePoint point;
            point.position = current;
            point.distance = totalDistance;
            point.curvature = calculateSignedCurvature(controlPoints, t);
            profile.push_back(point);
            lastT = t;
        }

        // Resample points at fixed distance intervals
        std::vector<ProfilePoint> resampledProfile;
        for (double d = 0; d <= totalDistance; d += DISTANCE_STEP) {
            auto it = std::lower_bound(profile.begin(), profile.end(), d,
                [](const ProfilePoint& point, double dist) {
                    return point.distance < dist;
                });

            if (it != profile.begin() && it != profile.end()) {
                auto prev = std::prev(it);
                double t = (d - prev->distance) / (it->distance - prev->distance);

                ProfilePoint point;
                point.position = prev->position * (1 - t) + it->position * t;
                point.distance = d;
                point.curvature = prev->curvature * (1 - t) + it->curvature * t;
                resampledProfile.push_back(point);
            }
        }

        // Forward pass
        for (size_t i = 0; i < resampledProfile.size(); i++) {
            double curvatureVel = maxVelocity / (1.0 + std::abs(resampledProfile[i].curvature * trackWidth));

            if (i == 0) {
                resampledProfile[i].velocity = 0;
            }
            else {
                double ds = DISTANCE_STEP;
                double v_prev = resampledProfile[i - 1].velocity;
                double v_max = std::sqrt(v_prev * v_prev + 2 * maxAccel * ds);
                resampledProfile[i].velocity = std::min({ v_max, curvatureVel, maxVelocity });
            }
        }

        // Backward pass
        for (int i = resampledProfile.size() - 1; i >= 0; i--) {
            double curvatureVel = maxVelocity / (1.0 + std::abs(resampledProfile[i].curvature * trackWidth));

            if (i == resampledProfile.size() - 1) {
                resampledProfile[i].velocity = 0;
            }
            else {
                double ds = DISTANCE_STEP;
                double v_next = resampledProfile[i + 1].velocity;
                double v_max = std::sqrt(v_next * v_next + 2 * maxDecel * ds);
                resampledProfile[i].velocity = std::min(resampledProfile[i].velocity, v_max);
                resampledProfile[i].velocity = std::min(resampledProfile[i].velocity, curvatureVel);
            }

            // Calculate differential drive velocities using signed curvature
            double centerVel = resampledProfile[i].velocity;
            resampledProfile[i].leftVelocity = centerVel * (1 - resampledProfile[i].curvature * trackWidth / 2);
            resampledProfile[i].rightVelocity = centerVel * (1 + resampledProfile[i].curvature * trackWidth / 2);
        }

        return resampledProfile;
    }
};