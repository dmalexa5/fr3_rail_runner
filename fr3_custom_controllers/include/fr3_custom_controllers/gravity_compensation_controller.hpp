#ifndef FR3_CUSTOM_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define FR3_CUSTOM_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_

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

#include <controller_interface/controller_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_custom_controllers {

  class GravityCompensationController : public controller_interface::ControllerInterface 
  {

    public:
      GravityCompensationController();

      [[nodiscard]] controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      [[nodiscard]] controller_interface::InterfaceConfiguration state_interface_configuration() const override;

      controller_interface::return_type update(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

      controller_interface::CallbackReturn on_init() override;

      controller_interface::CallbackReturn on_configure(
        const rclcpp_lifecycle::State & previous_state) override;

      controller_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;


    private:
      
      std::vector<std::string> joint_names_;
      std::vector<std::string> command_interface_types_;
      std::vector<std::string> state_interface_types_;
      
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_effort_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;

      // std::unordered_map<
      //   std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
      //   command_interface_map_ = {
      //     {"effort", &joint_effort_command_interface_}};

      // std::unordered_map<
      //   std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
      //   state_interface_map_ = {
      //     {"position", &joint_position_state_interface_},
      //     {"velocity", &joint_velocity_state_interface_}};
      };

}

#endif