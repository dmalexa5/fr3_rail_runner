#include <fr3_custom_controllers/gravity_compensation_controller.hpp>

#include <exception>
#include <string>

namespace fr3_custom_controllers {

  controller_interface::InterfaceConfiguration GravityCompensationController::command_interface_configuration() const {
    controller_interface::InterfaceConfiguration config;
    config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

    //TODO: Change this hardcoded joint value
    config.names.push_back("fr3_joint_linear/effort");
    
    for (int i = 1; i <= num_joints - 1; ++i) {
      config.names.push_back(arm_id_ + "_joint" + std::to_string(i) + "/effort");
    }

    return config;
  }

  controller_interface::InterfaceConfiguration GravityCompensationController::state_interface_configuration() const {
    return {}; // Return a default-constructed object controller_interface::InterfaceConfiguration
  }

  controller_interface::return_type GravityCompensationController::update(
    const rclcpp::Time& /*time*/,
    const rclcpp::Duration& /*period*/) {
  for (auto& command_interface : command_interfaces_) {
    command_interface.set_value(0);
  }
  return controller_interface::return_type::OK;
  }

  CallbackReturn GravityCompensationController::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  arm_id_ = "fr3"; //TODO: Change hardcoded value
  
  return CallbackReturn::SUCCESS;
  }

  CallbackReturn GravityCompensationController::on_init() {
  try {
    auto_declare<std::string>("arm_id", "fr3");
  } catch (const std::exception& e) {
    fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
  }

} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::GravityCompensationController, controller_interface::ControllerInterface)