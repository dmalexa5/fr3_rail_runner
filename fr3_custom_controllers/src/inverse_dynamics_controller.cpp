#include <fr3_custom_controllers/inverse_dynamics_controller.hpp>
#include <fr3_custom_controllers/utils.hpp>

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

  for (size_t i = 0; i < joint_position_state_interface_.size(); ++i) {

    q[i] = joint_position_state_interface_[i].get().get_value();
    qd[i] = joint_velocity_state_interface_[i].get().get_value();

    
  }
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "q: %s", eigen_str(q).c_str());
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "qd: %s", eigen_str(qd).c_str());
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "q_des: %s", eigen_str(q_des).c_str());
  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "qd_des: %s", eigen_str(qd_des).c_str());

  //
  // τ=M(q)[q¨​des​+Kd​(q˙​des​−q˙​)+Kp​(qdes​−q)]+C(q,q˙​)q˙​+g(q)
  //

  pinocchio::computeAllTerms(model, data, q, qd);

  const double Kp = 300.0;
  const double Kd = 10.0;

  Eigen::VectorXd qdd_cmd = qdd_des + Kd * (qd_des - qd) + Kp * (q_des - q);

  // Eigen::MatrixXd M = data.M; // Inertia matrix M(q)
  // Eigen::VectorXd b = data.nle; // Nonlinear effects = C(q,qd)*qd + g(q)

  // Eigen::VectorXd tau = M * qdd_cmd + b;

  tau = pinocchio::rnea(model, data, q, qd, qdd_cmd);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500, "tau: %s", eigen_str(tau).c_str());

  for (size_t i = 0; i< joint_effort_command_interface_.size(); i++) {
    
    joint_effort_command_interface_[i].get().set_value(tau[i]);
    
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