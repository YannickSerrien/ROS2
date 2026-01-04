/**
 * @file kinematics.hpp
 * @brief Kinematics helper functions for 2-link planar arm
 *
 * Provides forward and inverse kinematics calculations
 */

#ifndef ROBOT_ARM_PROJECT__KINEMATICS_HPP_
#define ROBOT_ARM_PROJECT__KINEMATICS_HPP_

#include <cmath>
#include <utility>

namespace robot_arm_kinematics
{

/**
 * @brief Arm configuration parameters
 */
struct ArmConfig
{
    double link1_length;  // Length of first link (meters)
    double link2_length;  // Length of second link (meters)
    double joint1_min;    // Joint 1 minimum angle (radians)
    double joint1_max;    // Joint 1 maximum angle (radians)
    double joint2_min;    // Joint 2 minimum angle (radians)
    double joint2_max;    // Joint 2 maximum angle (radians)
};

/**
 * @brief Forward kinematics: joint angles → end-effector position
 *
 * @param theta1 Joint 1 angle (radians)
 * @param theta2 Joint 2 angle (radians)
 * @param config Arm configuration
 * @return std::pair<double, double> End-effector (x, y) position
 */
std::pair<double, double> forward_kinematics(
    double theta1,
    double theta2,
    const ArmConfig & config);

/**
 * @brief Inverse kinematics: end-effector position → joint angles
 *
 * @param target_x Target x position
 * @param target_y Target y position
 * @param config Arm configuration
 * @param elbow_up Use elbow-up solution (true) or elbow-down (false)
 * @param theta1 Output: Joint 1 angle
 * @param theta2 Output: Joint 2 angle
 * @return true if solution exists, false if unreachable
 */
bool inverse_kinematics(
    double target_x,
    double target_y,
    const ArmConfig & config,
    bool elbow_up,
    double & theta1,
    double & theta2);

/**
 * @brief Check if target position is reachable
 *
 * @param target_x Target x position
 * @param target_y Target y position
 * @param config Arm configuration
 * @return true if reachable, false otherwise
 */
bool is_reachable(
    double target_x,
    double target_y,
    const ArmConfig & config);

/**
 * @brief Interpolate between two angles with shortest path
 *
 * @param start_angle Starting angle (radians)
 * @param end_angle Ending angle (radians)
 * @param t Interpolation parameter [0, 1]
 * @return Interpolated angle
 */
double interpolate_angle(double start_angle, double end_angle, double t);

}  // namespace robot_arm_kinematics

#endif  // ROBOT_ARM_PROJECT__KINEMATICS_HPP_
