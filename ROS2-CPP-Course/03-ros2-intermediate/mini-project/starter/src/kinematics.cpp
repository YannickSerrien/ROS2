/**
 * @file kinematics.cpp
 * @brief Implementation of kinematics functions
 *
 * This file is PROVIDED COMPLETE to students
 * Focus on integrating these functions into ROS2 nodes
 */

#include "robot_arm_project/kinematics.hpp"
#include <cmath>

namespace robot_arm_kinematics
{

std::pair<double, double> forward_kinematics(
    double theta1,
    double theta2,
    const ArmConfig & config)
{
    double L1 = config.link1_length;
    double L2 = config.link2_length;

    // Calculate end-effector position
    double x = L1 * std::cos(theta1) + L2 * std::cos(theta1 + theta2);
    double y = L1 * std::sin(theta1) + L2 * std::sin(theta1 + theta2);

    return {x, y};
}

bool inverse_kinematics(
    double target_x,
    double target_y,
    const ArmConfig & config,
    bool elbow_up,
    double & theta1,
    double & theta2)
{
    double L1 = config.link1_length;
    double L2 = config.link2_length;

    // Distance to target
    double r_squared = target_x * target_x + target_y * target_y;
    double r = std::sqrt(r_squared);

    // Check reachability
    if (r > L1 + L2 - 0.001 || r < std::abs(L1 - L2) + 0.001) {
        return false;  // Unreachable
    }

    // Calculate theta2 using law of cosines
    double cos_theta2 = (r_squared - L1 * L1 - L2 * L2) / (2.0 * L1 * L2);

    // Clamp to valid range (numerical stability)
    cos_theta2 = std::max(-1.0, std::min(1.0, cos_theta2));

    // Two solutions: elbow up (+) or elbow down (-)
    theta2 = elbow_up ? std::acos(cos_theta2) : -std::acos(cos_theta2);

    // Calculate theta1
    double k1 = L1 + L2 * std::cos(theta2);
    double k2 = L2 * std::sin(theta2);

    theta1 = std::atan2(target_y, target_x) - std::atan2(k2, k1);

    // Normalize angles to [-π, π]
    while (theta1 > M_PI) theta1 -= 2.0 * M_PI;
    while (theta1 < -M_PI) theta1 += 2.0 * M_PI;
    while (theta2 > M_PI) theta2 -= 2.0 * M_PI;
    while (theta2 < -M_PI) theta2 += 2.0 * M_PI;

    // Check joint limits
    if (theta1 < config.joint1_min || theta1 > config.joint1_max ||
        theta2 < config.joint2_min || theta2 > config.joint2_max)
    {
        return false;
    }

    return true;
}

bool is_reachable(
    double target_x,
    double target_y,
    const ArmConfig & config)
{
    double L1 = config.link1_length;
    double L2 = config.link2_length;

    double r = std::sqrt(target_x * target_x + target_y * target_y);

    // Check if within workspace
    return (r <= L1 + L2 - 0.001) && (r >= std::abs(L1 - L2) + 0.001);
}

double interpolate_angle(double start_angle, double end_angle, double t)
{
    // Find shortest path
    double diff = end_angle - start_angle;

    // Normalize difference to [-π, π]
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;

    return start_angle + t * diff;
}

}  // namespace robot_arm_kinematics
