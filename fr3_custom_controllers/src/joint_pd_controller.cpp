#include <fr3_custom_controllers/joint_pd_controller.hpp>

#include <string>
#include <vector>
#include <exception>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace fr3_custom_controllers {

  JointPDController::JointPDController() : controller_interface::ControllerInterface() {}

  controller_interface::CallbackReturn JointPDController::on_init()
  {

    try {
      joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);

      command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    
      state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    RCLCPP_INFO(get_node()->get_logger(), "Controller has been initialized.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointPDController::on_configure(const rclcpp_lifecycle::State &)
  {

    RCLCPP_INFO(get_node()->get_logger(), "Automatically confirming configuration with no issues.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointPDController::on_activate(const rclcpp_lifecycle::State &)
  {

    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();

    for (auto & interface : command_interfaces_)
    {
      if (interface.get_interface_name() == hardware_interface::HW_IF_EFFORT)
        joint_effort_command_interface_.push_back(std::ref(interface));
    }

    for (auto & interface : state_interfaces_)
    {
      if (interface.get_interface_name() == hardware_interface::HW_IF_POSITION)
        joint_position_state_interface_.push_back(std::ref(interface));
      if (interface.get_interface_name() == hardware_interface::HW_IF_VELOCITY)
        joint_velocity_state_interface_.push_back(std::ref(interface));
    }

    RCLCPP_INFO(get_node()->get_logger(), "JointPDController activated with %zu joints",
                joint_effort_command_interface_.size());

    return CallbackReturn::SUCCESS;
  }


  controller_interface::InterfaceConfiguration JointPDController::command_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {}
    };

    config.names.reserve(joint_names_.size() * command_interface_types_.size());

    for (const auto & joint_name : joint_names_)
    {
      for (const auto & interface_type : command_interface_types_)
      {
        config.names.push_back(joint_name + "/" + interface_type);
      }
    }

    RCLCPP_INFO(get_node()->get_logger(), "Command interfaces have been configured.");

    return config;
  }

  controller_interface::InterfaceConfiguration JointPDController::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration config = {
      controller_interface::interface_configuration_type::INDIVIDUAL, {}
    };

    config.names.reserve(joint_names_.size() * state_interface_types_.size());
    for (const auto & joint_name : joint_names_)
    {
      for (const auto & interface_type : state_interface_types_)
      {
        config.names.push_back(joint_name + "/" + interface_type);
      }
    }

    RCLCPP_INFO(get_node()->get_logger(), "State interfaces have been configured.");

    return config;
  }

  controller_interface::return_type JointPDController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
  )
  {
    for (size_t i = 0; i < joint_position_state_interface_.size(); i++) {
      q = joint_position_state_interface_[i].get().get_value();
      qd = joint_velocity_state_interface_[i].get().get_value();

      tau = -3000 * q + -100 * qd;
      
      RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 100,
        "Got joint %zu position %.2f, velocity %.2f, and commanded effort %.2f",
        i, q, qd, tau);

      joint_effort_command_interface_[i].get().set_value(tau);

    }

    return controller_interface::return_type::OK;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::JointPDController, controller_interface::ControllerInterface)