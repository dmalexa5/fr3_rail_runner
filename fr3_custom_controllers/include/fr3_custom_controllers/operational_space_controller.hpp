#ifndef FR3_CUSTOM_CONTROLLERS__OPERATIONAL_SPACE_CONTROLLER_HPP_
#define FR3_CUSTOM_CONTROLLERS__OPERATIONAL_SPACE_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

#include "controller_laws/osc_law.hpp"

constexpr int N_JOINTS = 8;
constexpr int M_TASK   = 6;

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_custom_controllers {

  class OperationalSpaceController : public controller_interface::ControllerInterface 
  {

    public:
      OperationalSpaceController();

      [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      controller_interface::CallbackReturn on_init() override;

      controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
        
    private:

      rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_subscriber;
      realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>> joint_msg_external_ptr;
      std::atomic<bool> new_msg_ = false;
      std::shared_ptr<std_msgs::msg::Float64MultiArray> joint_msg_;
      
      
      std::vector<std::string> joint_names_;
      std::vector<std::string> command_interface_types_;
      std::vector<std::string> state_interface_types_;
      
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_effort_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;

      // Dynamics
      Eigen::Matrix<double, N_JOINTS, N_JOINTS> M;
      Vector8d C, G;

      //Kinematics
      Matrix68 J, Jdot;

      // Task-space
      Eigen::Matrix<double, M_TASK, 1> x, xdot, x_des, xdot_des, xddot_des;

      //Gains
      Matrix66 Kp, Kd;
      };

}

#endif