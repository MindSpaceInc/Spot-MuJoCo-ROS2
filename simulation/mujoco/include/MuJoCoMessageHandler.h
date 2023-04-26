#pragma once

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rmw/types.h>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>

#include "communication/msg/actuator_cmds.hpp"
#include "communication/msg/touch_sensor.hpp"
#include "communication/srv/simulation_reset.hpp"

#include "array_safety.h"
#include "simulate.h"

using namespace rclcpp;

using namespace std::chrono_literals;

namespace deepbreak {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

class MuJoCoMessageHandler : public rclcpp::Node {
public:
  struct ActuatorCmds {
    double time = 0.0;
    std::vector<std::string> actuators_name;
    std::vector<float> kp;
    std::vector<float> pos;
    std::vector<float> kd;
    std::vector<float> vel;
    std::vector<float> torque;
  };

  MuJoCoMessageHandler(mj::Simulate *sim);
  ~MuJoCoMessageHandler();

  std::shared_ptr<ActuatorCmds> get_actuator_cmds_ptr();

private:
  void reset_callback(
      const std::shared_ptr<communication::srv::SimulationReset::Request> request,
      std::shared_ptr<communication::srv::SimulationReset::Response> response);

  void imu_callback();

  void odom_callback();

  void touch_callback();

  void joint_callback();

  void img_callback();

  void actuator_cmd_callback(
      const communication::msg::ActuatorCmds::SharedPtr msg) const;

  void parameter_callback(const rclcpp::Parameter &);

  void drop_old_message();

  void throw_box();

  mj::Simulate *sim_;
  std::string name_prefix, model_param_name;
  std::vector<rclcpp::TimerBase::SharedPtr> timers_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr
      joint_state_publisher_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<communication::msg::TouchSensor>::SharedPtr touch_publisher_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr depth_img_publisher_ptr_;
  Publisher<sensor_msgs::msg::Image>::SharedPtr rgb_img_publisher_ptr_;

  rclcpp::Subscription<communication::msg::ActuatorCmds>::SharedPtr
      actuator_cmd_subscription_;
  rclcpp::Service<communication::srv::SimulationReset>::SharedPtr reset_service_;

  std::shared_ptr<rclcpp::ParameterEventHandler> param_subscriber_;

  std::shared_ptr<rclcpp::ParameterCallbackHandle> cb_handle_;

  std::shared_ptr<ActuatorCmds> actuator_cmds_ptr_;

  std::thread spin_thread;
};

} // namespace deepbreak
