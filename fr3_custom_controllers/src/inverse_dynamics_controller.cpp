#include <fr3_custom_controllers/inverse_dynamics_controller.hpp>

#include <string>
#include <vector>
#include <exception>
#include <iostream>
#include <fstream>
#include <chrono>
#include <thread>
#include <iterator>
#include <sstream>

#include "rclcpp/qos.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>
#include <pinocchio/algorithm/model.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>

namespace fr3_custom_controllers {

  std::string loadURDF(const std::string& path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  InverseDynamicsController::InverseDynamicsController() : controller_interface::ControllerInterface(),
      model(),
      data(model)
  {

  }

  controller_interface::CallbackReturn InverseDynamicsController::on_init()
  {

    try {
      joint_names_ = auto_declare<std::vector<std::string>>("joints", joint_names_);
      num_joints = joint_names_.size();

      command_interface_types_ =
        auto_declare<std::vector<std::string>>("command_interfaces", command_interface_types_);
    
      state_interface_types_ =
        auto_declare<std::vector<std::string>>("state_interfaces", state_interface_types_);

    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    try {
      robot_description = loadURDF("/home/dmalexa5/debug_ws/src/test_controllers/urdf/fr3.urdf");

      pinocchio::Model arm_model;
      pinocchio::urdf::buildModelFromXML(robot_description, arm_model);

      std::vector<pinocchio::JointIndex>
        frozen_joints = {
          arm_model.getJointId("fr3_finger_joint1"),
          arm_model.getJointId("fr3_finger_joint2")
        };

      Eigen::VectorXd q0 = Eigen::VectorXd::Zero(arm_model.nq);

      for (size_t i = 0; i < frozen_joints.size(); i++) {
        const int q_index = arm_model.joints[frozen_joints[i]].idx_q(); // index in q
        q0[q_index] = 0.0;
      }
      
      // arm only pinocchio model
      model = pinocchio::buildReducedModel(arm_model, frozen_joints, q0);
      data = pinocchio::Data(model);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load debug urdf: %s \n", e.what());
      return CallbackReturn::ERROR;
    }


    RCLCPP_INFO(get_node()->get_logger(), "Controller has been initialized.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn InverseDynamicsController::on_configure(const rclcpp_lifecycle::State &)
  {
    
    q = Vector8d::Zero();
    q_des = Vector8d::Zero();

    qd = Vector8d::Zero();
    qd_des = Vector8d::Zero();

    qdd = Vector8d::Zero();
    qdd_des = Vector8d::Zero();

    effort_limits = Vector8d::Constant(80.0);

    auto callback = [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg_) -> void
    {
      RCLCPP_INFO(get_node()->get_logger(), "Received new joint positions.");
      joint_msg_external_ptr.writeFromNonRT(msg_);
      new_msg_ = true;
    };

    joint_command_subscriber =
    get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
      "~/joint_command", rclcpp::SystemDefaultsQoS(), callback);

    RCLCPP_INFO(get_node()->get_logger(), "Subscription to ~/joint_command created succesffully.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn InverseDynamicsController::on_activate(const rclcpp_lifecycle::State &)
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

    RCLCPP_INFO(get_node()->get_logger(), "InverseDynamicsController activated with %zu joints",
                joint_effort_command_interface_.size());

    return CallbackReturn::SUCCESS;
  }


  controller_interface::InterfaceConfiguration InverseDynamicsController::command_interface_configuration() const
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

  controller_interface::InterfaceConfiguration InverseDynamicsController::state_interface_configuration() const
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

 controller_interface::return_type InverseDynamicsController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Ensure vectors match model sizes
  if (q.size()     != model.nq)  q.resize(model.nq);
  if (q_des.size() != model.nq)  q_des.resize(model.nq);
  if (qd.size()    != model.nv)  qd.resize(model.nv);
  if (qd_des.size() != model.nv) qd_des.resize(model.nv);
  if (qdd_des.size() != model.nv) qdd_des.resize(model.nv);

  // initialize to zero so unmapped indices are safe
  q.setZero();
  qd.setZero();

  // Read hardware state BUT place values into pinocchio indices (idx_q / idx_v)
  for (size_t i = 0; i < joint_position_state_interface_.size(); ++i) {
    const std::string &name = joint_names_[i];
    const auto jid = model.getJointId(name);
    
    if (jid >= model.joints.size()) {
      RCLCPP_WARN(get_node()->get_logger(), "Joint %s not found in model", name.c_str());
      continue;
    }

    const int iq = pinocchio::idx_q(model.joints[jid]); // index into q
    const int iv = pinocchio::idx_v(model.joints[jid]); // index into qd / tau

    if (iq >= 0 && iq < (int)q.size())
      q[iq] = joint_position_state_interface_[i].get().get_value();

    if (iv >= 0 && iv < (int)qd.size())
      qd[iv] = joint_velocity_state_interface_[i].get().get_value();
  }

  // FK and inverse dynamics (we assume q_des / qd_des / qdd_des are prepared elsewhere)
  pinocchio::forwardKinematics(model, data, q, qd);
  tau = pinocchio::rnea(model, data, q_des, qd_des, qdd_des);

  // Safe logging of first up-to-8 tau entries (won't crash if tau is shorter)
  double tvals[8] = {0.,0.,0.,0.,0.,0.,0.,0.};
  for (size_t k = 0; k < 8 && k < tau.size(); ++k) tvals[k] = tau[k];

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "Tau size: %zu \nGot desired torques [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]\ncmd interface size: %zu",
    tau.size(), tvals[0], tvals[1], tvals[2], tvals[3], tvals[4], tvals[5], tvals[6], tvals[7],
    joint_effort_command_interface_.size());

  // Write commands: map controller joint i -> pinocchio tau at idx_v
  for (size_t i = 0; i < joint_effort_command_interface_.size(); ++i) {
    const std::string &name = joint_names_[i];

    // Special-case user override for fr3_joint1
    if (name == "fr3_joint1") {
      joint_effort_command_interface_[i].get().set_value(8.0);
      continue;
    }

    const auto jid = model.getJointId(name);
    if (jid >= model.joints.size()) {
      RCLCPP_WARN(get_node()->get_logger(), "Joint %s not found in model", name.c_str());
      joint_effort_command_interface_[i].get().set_value(0.0);
      continue;
    }

    const int iv = pinocchio::idx_v(model.joints[jid]);
    if (iv >= 0 && iv < (int)tau.size()) {
      joint_effort_command_interface_[i].get().set_value(tau[iv]);
    } else {
      RCLCPP_WARN(get_node()->get_logger(), "Invalid idx_v for joint %s (iv=%d)", name.c_str(), iv);
      joint_effort_command_interface_[i].get().set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}


  controller_interface::CallbackReturn InverseDynamicsController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::InverseDynamicsController, controller_interface::ControllerInterface)