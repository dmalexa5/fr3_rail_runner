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
    try 
    {
      joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
      command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
      state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);
    } 
    catch (const std::exception& e) 
    {
      RCLCPP_ERROR(get_node()->get_logger(), "Could not initialize joints!");
      return CallbackReturn::ERROR;
    }

    // try
    // {
    //   for (const auto &joint_name : joint_names_) {
    //     auto p = auto_declare<double>("gains." + joint_name + ".p", 0.0);
    //     auto d = auto_declare<double>("gains." + joint_name + ".d", 0.0);

    //     RCLCPP_INFO(get_node()->get_logger(), "Got gains for %s - p: %f d: %f", joint_name.c_str(), p, d);

    //     joint_gains_[joint_name] = {p, d};
    //   }
    // }
    // catch(const std::exception& e)
    // {
    //   RCLCPP_ERROR(get_node()->get_logger(), "Unable to find gains. Are they defined in the controller config file?");
    //   return CallbackReturn::ERROR;
    // }
    
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

    return config;
  }

  controller_interface::CallbackReturn JointPDController::on_configure(const rclcpp_lifecycle::State &)
  {

    // std::vector<double*> p_gains_;
    // std::vector<double*> d_gains_;

    // for (const auto &joint_name : joint_names_) {
    //   p_gains_.push_back(&joint_gains_[joint_name].p);
    //   d_gains_.push_back(&joint_gains_[joint_name].d);
    // }
    
    RCLCPP_INFO(get_node()->get_logger(), "Automatically confirming configuration with no issues.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointPDController::on_activate(const rclcpp_lifecycle::State &)
  {
    
    // clear out vectors in case of restart
    joint_effort_command_interface_.clear();
    joint_position_state_interface_.clear();
    joint_velocity_state_interface_.clear();

    // assign command interfaces
    for (auto & interface : command_interfaces_)
    {
      command_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    // assign state interfaces
    for (auto & interface : state_interfaces_)
    {
      state_interface_map_[interface.get_interface_name()]->push_back(interface);
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn JointPDController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    RCLCPP_INFO(get_node()->get_logger(), "Deactivating controller, zeroing efforts...");

    for (size_t i = 0; i < joint_effort_command_interface_.size(); ++i)
    {
      try
      {
        joint_effort_command_interface_[i].get().set_value(0.0);
      }
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set effort for joint index %zu: %s", i, e.what());
      }
    }

    return CallbackReturn::SUCCESS;
  }

  controller_interface::return_type JointPDController::update(
    const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/
  )
  {
    for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
    {
      try {
        joint_effort_command_interface_[i].get().set_value(3); // set all commands to zero
      } catch (const std::exception& e) {
        RCLCPP_ERROR(get_node()->get_logger(), "Failed to set effort for index %ld: %s", i, e.what());
      }
    }
    // for (size_t i = 0; i < joint_effort_command_interface_.size(); i++)
    // {

    //   q = joint_position_state_interface_[i].get().get_value();
    //   qd = joint_velocity_state_interface_[i].get().get_value();

    //   tau = std::clamp(3000 * q + 100 * qd, -100.0, 100.0); // desired joint positions are all zero



    //   try
    //   {
    //     joint_effort_command_interface_[i].get().set_value(tau);
    //   }
    //   catch(const std::exception& e)
    //   {
    //     RCLCPP_ERROR(get_node()->get_logger(), "Failed to set effort value for index %ld", i);
    //   }
    // }

    return controller_interface::return_type::OK;
  }

} 

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::JointPDController, controller_interface::ControllerInterface)