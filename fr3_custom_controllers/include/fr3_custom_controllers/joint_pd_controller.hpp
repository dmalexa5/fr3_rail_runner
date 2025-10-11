#ifndef FR3_CUSTOM_CONTROLLERS__JOINT_PD_CONTROLLER_HPP_
#define FR3_CUSTOM_CONTROLLERS__JOINT_PD_CONTROLLER_HPP_

#include <atomic>
#include <chrono>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

#include <controller_interface/controller_interface.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_custom_controllers {
  
  struct JointGains {
    double p;
    double d;
  };

  class JointPDController : public controller_interface::ControllerInterface 
  {

    public:
      JointPDController();

      using Vector8d = Eigen::Matrix<double, 8, 1>;

      controller_interface::InterfaceConfiguration command_interface_configuration() const override;

      controller_interface::InterfaceConfiguration state_interface_configuration() const override;

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
      
      double q; // current joint position
      double qd; // current joint velocity

      double tau; //output torque



      std::unordered_map<std::string, JointGains> joint_gains_;

      std::vector<std::string> joint_names_;
      std::vector<std::string> command_interface_types_;
      std::vector<std::string> state_interface_types_;
      
      std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>>
        joint_effort_command_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_position_state_interface_;
      std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>
        joint_velocity_state_interface_;

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedCommandInterface>> *>
        command_interface_map_ = {
          {"effort", &joint_effort_command_interface_}};

      std::unordered_map<
        std::string, std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>> *>
        state_interface_map_ = {
          {"position", &joint_position_state_interface_},
          {"velocity", &joint_velocity_state_interface_}};
      };

}

#endif