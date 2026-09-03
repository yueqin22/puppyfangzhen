#include "puppy_gait/gait_controller.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

namespace puppy_gait {

GaitController::GaitController()
    : Node("gait_controller"),
      gait_(DEFAULT_GAIT),
      ik_({0.105, 0.115}),
      t_(0.0),
      vx_(0.0), vy_(0.0), wz_(0.0),
      standing_(true),
      control_rate_(100.0) {

    // Declare parameters
    this->declare_parameter("control_rate", 100.0);
    this->declare_parameter("gait_period", 0.5);
    this->declare_parameter("step_length", 0.08);
    this->declare_parameter("step_height", 0.05);
    this->declare_parameter("body_height", 0.205);
    this->declare_parameter("thigh_length", 0.105);
    this->declare_parameter("calf_length", 0.115);
    // Hip offsets set the lever arm for turning; they must match the URDF.
    this->declare_parameter("hip_half_length", DEFAULT_GAIT.half_length);
    this->declare_parameter("hip_half_width", DEFAULT_GAIT.half_width);

    control_rate_ = this->get_parameter("control_rate").as_double();

    // Update gait params from ROS parameters
    GaitParams params = DEFAULT_GAIT;
    params.period = this->get_parameter("gait_period").as_double();
    params.step_length = this->get_parameter("step_length").as_double();
    params.step_height = this->get_parameter("step_height").as_double();
    params.body_height = this->get_parameter("body_height").as_double();
    params.half_length = this->get_parameter("hip_half_length").as_double();
    params.half_width = this->get_parameter("hip_half_width").as_double();
    gait_.setParams(params);

    if (params.half_length <= 0.0 || params.half_width <= 0.0) {
      RCLCPP_WARN(this->get_logger(),
                  "hip_half_length/hip_half_width must be positive (got %.3f, "
                  "%.3f); turning and strafing will be wrong.",
                  params.half_length, params.half_width);
    }
    ik_ = InverseKinematics({
        this->get_parameter("thigh_length").as_double(),
        this->get_parameter("calf_length").as_double()});

    // Joint names in the order expected by joint_trajectory_controller
    joint_names_ = {
        "FR_hip_yaw_joint", "FR_hip_pitch_joint", "FR_knee_joint",
        "FL_hip_yaw_joint", "FL_hip_pitch_joint", "FL_knee_joint",
        "RR_hip_yaw_joint", "RR_hip_pitch_joint", "RR_knee_joint",
        "RL_hip_yaw_joint", "RL_hip_pitch_joint", "RL_knee_joint",
    };

    // Subscriber for velocity commands
    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10,
        std::bind(&GaitController::cmdVelCallback, this, std::placeholders::_1));

    // Publisher for joint trajectory
    traj_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
        "/joint_trajectory_controller/joint_trajectory", 10);

    // Control timer
    auto period = std::chrono::microseconds(
        static_cast<int>(1e6 / control_rate_));
    timer_ = this->create_wall_timer(period,
        std::bind(&GaitController::timerCallback, this));

    RCLCPP_INFO(this->get_logger(),
                "Gait controller started. Control rate: %.1f Hz, Period: %.2f s",
                control_rate_, params.period);
}

void GaitController::cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
    vx_ = msg->linear.x;
    vy_ = msg->linear.y;
    wz_ = msg->angular.z;

    // If any velocity command is non-zero, start walking
    double speed = std::sqrt(vx_ * vx_ + vy_ * vy_ + wz_ * wz_);
    standing_ = (speed < 0.01);

    if (standing_) {
        RCLCPP_DEBUG(this->get_logger(), "Standing mode");
    } else {
        RCLCPP_DEBUG(this->get_logger(),
                     "Walking: vx=%.2f vy=%.2f wz=%.2f", vx_, vy_, wz_);
    }
}

void GaitController::timerCallback() {
    // Advance gait time
    double dt = 1.0 / control_rate_;
    t_ += dt;

    publishJointTrajectory();
}

void GaitController::publishJointTrajectory() {
    auto msg = trajectory_msgs::msg::JointTrajectory();
    msg.joint_names = {joint_names_.begin(), joint_names_.end()};

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions.resize(12);

    std::array<FootPosition, 4> feet;

    if (standing_) {
        // Standing pose: all feet at default position
        feet = gait_.getStandingPose();
    } else {
        // Walking: generate foot positions from gait
        feet = gait_.getAllFootPositions(t_, vx_, vy_, wz_);
    }

    // Solve IK for each leg
    for (int i = 0; i < 4; i++) {
        JointAngles angles = ik_.solve(feet[i]);

        // Map to joint indices: each leg has 3 joints
        int base = i * 3;
        point.positions[base + 0] = angles.hip_yaw;
        point.positions[base + 1] = angles.hip_pitch;
        point.positions[base + 2] = angles.knee;
    }

    // Set time from start (short horizon for real-time control)
    point.time_from_start.sec = 0;
    point.time_from_start.nanosec = static_cast<uint32_t>(1e9 / control_rate_);

    msg.points.push_back(point);

    traj_pub_->publish(msg);
}

}  // namespace puppy_gait

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<puppy_gait::GaitController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
