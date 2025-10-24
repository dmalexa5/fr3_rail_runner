#include <fr3_custom_controllers/operational_space_controller.hpp>
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
#include <geometry_msgs/msg/pose.hpp>

namespace fr3_custom_controllers {


  OperationalSpaceController::OperationalSpaceController() : controller_interface::ControllerInterface(),
      model(),
      data(model)
  {

  }

  controller_interface::CallbackReturn OperationalSpaceController::on_init()
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
      
      ee_name = "fr3_link8";
      model = pinocchio::buildReducedModel(arm_model, frozen_joints, q0); // arm only pinocchio model
      data = pinocchio::Data(model);
      ee_frame_idx = model.getFrameId(ee_name, pinocchio::FrameType::BODY);
    }
    catch (const std::exception& e) {
      RCLCPP_ERROR(get_node()->get_logger(), "Failed to load debug urdf: %s \n", e.what());
      return CallbackReturn::ERROR;
    }


    RCLCPP_INFO(get_node()->get_logger(), "Controller has been initialized.");

    return CallbackReturn::SUCCESS;
  }

  controller_interface::CallbackReturn OperationalSpaceController::on_configure(const rclcpp_lifecycle::State &)
  {
    // allocate & zero
    M.resize(num_joints, num_joints);       M.setZero();
    Minv.resize(num_joints, num_joints);    Minv.setZero();
    b.resize(num_joints);                   b.setZero();

    J.resize(6, num_joints);                J.setZero();
    Jtrans.resize(num_joints, 6);           Jtrans.setZero();
    JJtrans.resize(6, 6);                   JJtrans.setZero();
    Jpseudo.resize(num_joints, 6);          Jpseudo.setZero();
    Jdot.resize(6, num_joints);             Jdot.setZero();

    lambda.resize(6, 6);                    lambda.setZero();
    Jdyncons.resize(num_joints, 6);         Jdyncons.setZero();

    // current pose set to identity transform (replace later with input pose)
    geometry_msgs::msg::Pose pose_msg;
    pose_msg.position.x = 0.26;
    pose_msg.position.y = 0.43;
    pose_msg.position.z = 0.58;
    pose_msg.orientation.x = 0.9996;
    pose_msg.orientation.y = -0.0024;
    pose_msg.orientation.z = 0.0198;
    pose_msg.orientation.w = 0.0219;

    pose_cur = pose_to_SE3(pose_msg);

    // gains (currently hardcoded)
    kp = 300.0;
    kd = 10.0;

    // 6D vectors
    x_err.setZero();
    xd.setZero();
    xdd_cmd.setZero();

    // fixed-size 8x8 identity
    I = Eigen::MatrixXd::Identity(8, 8);

    // null-space torque as (n x 1) matrix
    null_space_tau.resize(num_joints, 1);
    null_space_tau.setZero();


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

  controller_interface::CallbackReturn OperationalSpaceController::on_activate(const rclcpp_lifecycle::State &)
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

    RCLCPP_INFO(get_node()->get_logger(), "OperationalSpaceController activated with %zu joints",
                joint_effort_command_interface_.size());

    return CallbackReturn::SUCCESS;
  }


  controller_interface::InterfaceConfiguration OperationalSpaceController::command_interface_configuration() const
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

  controller_interface::InterfaceConfiguration OperationalSpaceController::state_interface_configuration() const
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

 controller_interface::return_type OperationalSpaceController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{

  for (size_t i = 0; i < joint_position_state_interface_.size(); ++i) {

    q[i] = joint_position_state_interface_[i].get().get_value();
    qd[i] = joint_velocity_state_interface_[i].get().get_value();
    
  }

  osc(q, qd, pose_des, tau);

  RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "Computed tau: [%.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f, %.2f]",
    tau[0], tau[1], tau[2], tau[3], tau[4], tau[5], tau[6], tau[7]);

  for (size_t i = 0; i< joint_effort_command_interface_.size(); i++) {
    
    joint_effort_command_interface_[i].get().set_value(tau[i]);
    
  }

  return controller_interface::return_type::OK;
  }

  void OperationalSpaceController::osc(
      const Vector8d& q,
      const Vector8d& qd,
      const pinocchio::SE3& pose_des,
      Vector8d& tau
  ) {

    pinocchio::computeAllTerms(model, data, q, qd);
    pinocchio::updateFramePlacements(model, data);
    pose_cur = data.oMf[ee_frame_idx];

    M = data.M;
    Minv = M.inverse();
    b = data.nle;

    // compute jacobian variations
    pinocchio::computeFrameJacobian(model, data, q, ee_frame_idx, J);

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Jacobian ---------\n %s", eigen_str(J).c_str());


    Jtrans = J.transpose();
    JJtrans = J * Jtrans; // intermediate matrix, probably will use cholesky decomp later
    Jpseudo = Jtrans * JJtrans.inverse();
    Jdot = pinocchio::computeJointJacobiansTimeVariation(model, data, q, qd);

    // task space inertia
    lambda = (J * Minv * Jtrans).inverse();
    
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "lambda ---------\n %s", eigen_str(lambda).c_str());

    // dynamically consistent jacobian
    Jdyncons = Minv * Jtrans * lambda;

    // task space acceleration
    computePoseError(pose_des, pose_cur, x_err);
    xd = pinocchio::getFrameVelocity(model, data, ee_frame_idx, pinocchio::WORLD).toVector();
    xdd_cmd = kp * x_err + kd * (-xd); // Desired velocity and acceleration are zero

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "xdd_cmd ---------\n %s", eigen_str(xdd_cmd).c_str());

    // null space effort
    null_space_tau = (I - Jtrans * Jpseudo.transpose()) * tau0;

    // computed torque
    tau = Jtrans * 
      (lambda * xdd_cmd + (lambda * (J * Minv * b - Jdot * qd)))
      + null_space_tau;
  }


  controller_interface::CallbackReturn OperationalSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::OperationalSpaceController, controller_interface::ControllerInterface)