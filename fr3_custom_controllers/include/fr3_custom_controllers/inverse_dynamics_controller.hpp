#ifndef FR3_CUSTOM_CONTROLLERS__INVERSE_DYNAMIC_CONTROLLER_HPP_
#define FR3_CUSTOM_CONTROLLERS__INVERSE_DYNAMIC_CONTROLLER_HPP_

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

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "realtime_tools/realtime_buffer.hpp"

#include "controller_interface/controller_interface.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_custom_controllers {
  using Vector8d = Eigen::Vector<double, 8>;

  class InverseDynamicsController : public controller_interface::ControllerInterface 
  {

    public:
      InverseDynamicsController();

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
      
      size_t num_joints;

      std::string robot_description;

      pinocchio::Model model;
      pinocchio::Data data;
      
      Vector8d q = Vector8d::Zero();
      Vector8d q_des = Vector8d::Zero();

      Vector8d qd = Vector8d::Zero();
      Vector8d qd_des = Vector8d::Zero();

      Vector8d qdd = Vector8d::Zero();
      Vector8d qdd_des = Vector8d::Zero();

      Vector8d effort_limits = Vector8d::Constant(80.0);
      Vector8d tau = Vector8d::Zero();

      };

}

#endif