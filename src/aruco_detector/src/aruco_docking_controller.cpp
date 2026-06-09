#include <memory>
#include <string>
#include <vector>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "geometry_msgs/msg/twist.hpp"
#include "aruco_msg/msg/aruco_docking.hpp"
#include "aruco_opencv_msgs/msg/aruco_detection.hpp"

class ArucoDockingController : public rclcpp::Node
{
public:
  ArucoDockingController() : Node("aruco_docking_controller")
  {
    declare_parameter<double>("kp", 0.5);
    declare_parameter<double>("ki", 0.01);
    declare_parameter<double>("kd", 0.1);
    declare_parameter<double>("control_rate", 20.0);
    declare_parameter<double>("error_tolerance", 0.02);
    declare_parameter<double>("max_linear_x", 0.5);
    declare_parameter<double>("kp_yaw", 1.0);
    declare_parameter<double>("ki_yaw", 0.01);
    declare_parameter<double>("kd_yaw", 0.05);
    declare_parameter<double>("max_angular_z", 0.3);

    kp_ = get_parameter("kp").as_double();
    ki_ = get_parameter("ki").as_double();
    kd_ = get_parameter("kd").as_double();
    control_rate_ = get_parameter("control_rate").as_double();
    error_tolerance_ = get_parameter("error_tolerance").as_double();
    max_linear_x_ = get_parameter("max_linear_x").as_double();
    kp_yaw_ = get_parameter("kp_yaw").as_double();
    ki_yaw_ = get_parameter("ki_yaw").as_double();
    kd_yaw_ = get_parameter("kd_yaw").as_double();
    max_angular_z_ = get_parameter("max_angular_z").as_double();

    cmd_vel_pub_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

    docking_sub_ = create_subscription<aruco_msg::msg::ArucoDocking>(
      "/cmd_aruco_docking", 1,
      std::bind(&ArucoDockingController::docking_callback, this, std::placeholders::_1)
    );

    detection_sub_ = create_subscription<aruco_opencv_msgs::msg::ArucoDetection>(
      "/aruco_detections", 1,
      std::bind(&ArucoDockingController::detection_callback, this, std::placeholders::_1)
    );

    control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate_),
      std::bind(&ArucoDockingController::control_loop, this)
    );

    RCLCPP_INFO(get_logger(), "ArucoDockingController started");
  }

private:
  void docking_callback(const aruco_msg::msg::ArucoDocking::SharedPtr msg)
  {
    target_id_ = msg->id;
    offset_z_ = msg->offset_z;
    active_ = true;
    integral_ = 0.0;
    prev_error_ = 0.0;
    integral_yaw_ = 0.0;
    prev_error_yaw_ = 0.0;
    last_valid_time_ = now();
    RCLCPP_INFO(get_logger(), "Docking command received: id=%d, offset_z=%.3f", target_id_, offset_z_);
  }

  void detection_callback(const aruco_opencv_msgs::msg::ArucoDetection::SharedPtr msg)
  {
    last_detection_ = msg;
  }

  void control_loop()
  {
    if (!active_) {
      return;
    }

    if (!last_detection_) {
      double elapsed = (now() - last_valid_time_).seconds();
      if (elapsed > 0.5) {
        stop_robot();
        active_ = false;
        RCLCPP_WARN(get_logger(), "No detections received, stopping docking");
      }
      return;
    }

    auto it = std::find_if(
      last_detection_->markers.begin(),
      last_detection_->markers.end(),
      [this](const aruco_opencv_msgs::msg::MarkerPose &m) {
        return m.marker_id == target_id_;
      }
    );

    if (it == last_detection_->markers.end()) {
      double elapsed = (now() - last_valid_time_).seconds();
      if (elapsed > 0.5) {
        stop_robot();
        active_ = false;
        RCLCPP_WARN(get_logger(), "Marker %d lost, stopping docking", target_id_);
      }
      return;
    }

    last_valid_time_ = now();

    double current_z = it->pose.position.z;
    double error = current_z - offset_z_;

    if (std::fabs(error) < error_tolerance_) {
      stop_robot();
      active_ = false;
      RCLCPP_INFO(get_logger(), "Docking complete: error=%.4f (tolerance=%.4f)", error, error_tolerance_);
      return;
    }

    double dt = 1.0 / control_rate_;

    // --- linear PID (z-distance) ---
    integral_ += error * dt;
    integral_ = std::clamp(integral_, -1.0, 1.0);
    double derivative = (error - prev_error_) / dt;
    double output = kp_ * error + ki_ * integral_ + kd_ * derivative;
    output = std::clamp(output, -max_linear_x_, max_linear_x_);
    prev_error_ = error;

    // --- yaw PID (from marker orientation quaternion) ---
    tf2::Quaternion q(
      it->pose.orientation.x,
      it->pose.orientation.y,
      it->pose.orientation.z,
      it->pose.orientation.w);
    tf2::Matrix3x3 rot(q);
    double error_yaw = std::atan2(rot[0][2], -rot[2][2]);

    integral_yaw_ += error_yaw * dt;
    integral_yaw_ = std::clamp(integral_yaw_, -1.0, 1.0);
    double derivative_yaw = (error_yaw - prev_error_yaw_) / dt;
    double output_yaw = kp_yaw_ * error_yaw + ki_yaw_ * integral_yaw_ + kd_yaw_ * derivative_yaw;
    output_yaw = std::clamp(output_yaw, -max_angular_z_, max_angular_z_);
    prev_error_yaw_ = error_yaw;

    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    twist->linear.x = output;
    twist->angular.z = output_yaw;
    cmd_vel_pub_->publish(std::move(twist));
  }

  void stop_robot()
  {
    auto twist = std::make_unique<geometry_msgs::msg::Twist>();
    cmd_vel_pub_->publish(std::move(twist));
  }

  double kp_, ki_, kd_, control_rate_, error_tolerance_, max_linear_x_;
  double kp_yaw_, ki_yaw_, kd_yaw_, max_angular_z_;
  int target_id_ = 0;
  double offset_z_ = 0.0;
  bool active_ = false;
  double integral_ = 0.0;
  double prev_error_ = 0.0;
  double integral_yaw_ = 0.0;
  double prev_error_yaw_ = 0.0;
  rclcpp::Time last_valid_time_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<aruco_msg::msg::ArucoDocking>::SharedPtr docking_sub_;
  rclcpp::Subscription<aruco_opencv_msgs::msg::ArucoDetection>::SharedPtr detection_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  aruco_opencv_msgs::msg::ArucoDetection::SharedPtr last_detection_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ArucoDockingController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
