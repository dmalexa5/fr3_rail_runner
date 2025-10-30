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
      base_name = "fr3_link_stationary";
      model = pinocchio::buildReducedModel(arm_model, frozen_joints, q0); // arm only pinocchio model
      data = pinocchio::Data(model);
      ee_frame_idx = model.getFrameId(ee_name, pinocchio::FrameType::BODY);
      base_frame_index = model.getFrameId(base_name, pinocchio::FrameType::FIXED_JOINT);
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

    // current desired pose (replace later with input pose)
    pose_des.position.x = 0.26;
    pose_des.position.y = 0.43;
    pose_des.position.z = 0.58;
    pose_des.orientation.x = 0.9996;
    pose_des.orientation.y = -0.0024;
    pose_des.orientation.z = 0.0198;
    pose_des.orientation.w = 0.0219;

    // gains (currently hardcoded)
    kp << 1000, 1000, 1000, 10, 10, 10;
    kd.setConstant(100);

    // 6D vectors
    x_err.setZero();
    xd.setZero();
    xdd_cmd.setZero();

    // fixed-size 8x8 identity
    I = Eigen::MatrixXd::Identity(8, 8);

    // null-space torque as (n x 1) matrix
    null_space_tau.resize(num_joints, 1);
    null_space_tau.setZero();


    // auto callback = [this](const std::shared_ptr<std_msgs::msg::Float64MultiArray> msg_) -> void
    // {
    //   RCLCPP_INFO(get_node()->get_logger(), "Received new joint positions.");
    //   joint_msg_external_ptr.writeFromNonRT(msg_);
    //   new_msg_ = true;
    // };

    // joint_command_subscriber =
    // get_node()->create_subscription<std_msgs::msg::Float64MultiArray>(
    //   "~/joint_command", rclcpp::SystemDefaultsQoS(), callback);

    // RCLCPP_INFO(get_node()->get_logger(), "Subscription to ~/joint_command created succesffully.");

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
      const geometry_msgs::msg::Pose& pose_des,
      Vector8d& tau
  ) {

    pinocchio::computeAllTerms(model, data, q, qd);
    pinocchio::updateFramePlacements(model, data);

    M = 0.5 * (data.M + data.M.transpose());
    Minv = M.inverse(); // TODO: Cholesky decomposition of M
    b = data.nle;

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Mass matrix ---------\n %s", eigen_str(M).c_str());

    // compute jacobian at end effector in world frame
    J = pinocchio::getFrameJacobian(model, data, ee_frame_idx, pinocchio::WORLD);
    Jtrans = J.transpose();
    
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "Jacobian ---------\n %s", eigen_str(J).c_str());

    // task space inertia
    lambda = (J * Minv * Jtrans).inverse();
    
    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "lambda ---------\n %s", eigen_str(lambda).c_str());

    // current end effector pose
    pose_cur = data.oMf[ee_frame_idx];

    // task space translation error
    trans_des << pose_des.position.x , pose_des.position.y, pose_des.position.z;
    trans_err = trans_des - pose_cur.translation();

    //task space orientation error
    quat_des.coeffs() << pose_des.orientation.x,
                        pose_des.orientation.y,
                        pose_des.orientation.z,
                        pose_des.orientation.w;
    quat_des.normalize();
    quat_err = Eigen::Quaterniond(pose_cur.rotation()) * quat_des.inverse();

    // task space pose error (using decomposed translation and orientation)
    x_err.topRows<3>() = trans_err;
    x_err.bottomRows<3>() = quat_err.toRotationMatrix().eulerAngles(2, 1, 0);


    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
    "x_err ---------\n %s", eigen_str(x_err).c_str());

    xd = pinocchio::getFrameVelocity(model, data, ee_frame_idx, pinocchio::WORLD).toVector();
    xdd_cmd = kp.asDiagonal() * x_err + kd.asDiagonal() * (-xd); // Desired velocity and acceleration are zero

    RCLCPP_INFO_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 500,
      "xdd_cmd ---------\n %s", eigen_str(xdd_cmd).c_str());

    // computed torque
    tau = Jtrans * lambda * xdd_cmd + b;
  }


  controller_interface::CallbackReturn OperationalSpaceController::on_deactivate(const rclcpp_lifecycle::State &)
  {
    release_interfaces();

    return CallbackReturn::SUCCESS;
  }

}

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(fr3_custom_controllers::OperationalSpaceController, controller_interface::ControllerInterface)