#pragma once

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "puppy_gait/trot_gait.hpp"
#include "puppy_gait/inverse_kinematics.hpp"

namespace puppy_gait {

/**
 * Gait controller ROS2 node.
 * Subscribes to /cmd_vel, generates joint trajectories using Trot gait + IK,
 * publishes to joint_trajectory_controller.
 */
class GaitController : public rclcpp::Node {
public:
    GaitController();

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void timerCallback();

    // Generate and publish joint trajectory
    void publishJointTrajectory();

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Gait and IK
    TrotGait gait_;
    InverseKinematics ik_;

    // State
    double t_;  // current gait time
    double vx_, vy_, wz_;  // velocity commands
    bool standing_;  // true if robot should stand still

    // Joint names (order: FR, FL, RR, RL x 3 joints)
    std::array<std::string, 12> joint_names_;

    // Parameters
    double control_rate_;
};

}  // namespace puppy_gait
