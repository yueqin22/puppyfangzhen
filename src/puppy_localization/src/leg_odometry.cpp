#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include <chrono>
#include <cmath>

using namespace std::chrono_literals;

class LegOdometry : public rclcpp::Node {
public:
    LegOdometry() : Node("leg_odometry"), x_(0.0), y_(0.0), theta_(0.0), last_time_(0) {
        // Subscribe to cmd_vel (velocity commands)
        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10,
            std::bind(&LegOdometry::cmdVelCallback, this, std::placeholders::_1));

        // Subscribe to IMU for yaw
        imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
            "/imu/data", 10,
            std::bind(&LegOdometry::imuCallback, this, std::placeholders::_1));

        // Publish odometry
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

        // TF broadcaster
        tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(this);

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(20ms, std::bind(&LegOdometry::timerCallback, this));

        last_time_ = this->now().seconds();

        RCLCPP_INFO(this->get_logger(), "Leg odometry node started");
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        vx_ = msg->linear.x;
        vy_ = msg->linear.y;
        wz_ = msg->angular.z;
    }

    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
        // Extract yaw from quaternion
        tf2::Quaternion q(
            msg->orientation.x,
            msg->orientation.y,
            msg->orientation.z,
            msg->orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        imu_yaw_ = yaw;
        has_imu_ = true;
    }

    void timerCallback() {
        double now = this->now().seconds();
        double dt = now - last_time_;
        last_time_ = now;

        if (dt <= 0 || dt > 1.0) return;

        // Use IMU yaw if available, otherwise integrate wz
        if (has_imu_) {
            theta_ = imu_yaw_;
        } else {
            theta_ += wz_ * dt;
        }

        // Integrate position
        x_ += (vx_ * std::cos(theta_) - vy_ * std::sin(theta_)) * dt;
        y_ += (vx_ * std::sin(theta_) + vy_ * std::cos(theta_)) * dt;

        // Publish odometry message
        nav_msgs::msg::Odometry odom;
        odom.header.stamp = this->now();
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;

        tf2::Quaternion q;
        q.setRPY(0, 0, theta_);
        odom.pose.pose.orientation.x = q.x();
        odom.pose.pose.orientation.y = q.y();
        odom.pose.pose.orientation.z = q.z();
        odom.pose.pose.orientation.w = q.w();

        odom.twist.twist.linear.x = vx_;
        odom.twist.twist.linear.y = vy_;
        odom.twist.twist.angular.z = wz_;

        odom_pub_->publish(odom);

        // Publish TF: odom -> base_link
        geometry_msgs::msg::TransformStamped tf_msg;
        tf_msg.header.stamp = this->now();
        tf_msg.header.frame_id = "odom";
        tf_msg.child_frame_id = "base_link";
        tf_msg.transform.translation.x = x_;
        tf_msg.transform.translation.y = y_;
        tf_msg.transform.translation.z = 0.0;
        tf_msg.transform.rotation.x = q.x();
        tf_msg.transform.rotation.y = q.y();
        tf_msg.transform.rotation.z = q.z();
        tf_msg.transform.rotation.w = q.w();

        tf_broadcaster_->sendTransform(tf_msg);
    }

    // ROS2 interfaces
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    rclcpp::TimerBase::SharedPtr timer_;

    // State
    double x_, y_, theta_;
    double vx_ = 0.0, vy_ = 0.0, wz_ = 0.0;
    double imu_yaw_ = 0.0;
    bool has_imu_ = false;
    double last_time_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LegOdometry>());
    rclcpp::shutdown();
    return 0;
}
