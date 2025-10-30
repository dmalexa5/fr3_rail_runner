#ifndef FR3_CUSTOM_CONTROLLERS__OPERATIONAL_SPACE_CONTROLLER_HPP_
#define FR3_CUSTOM_CONTROLLERS__OPERATIONAL_SPACE_CONTROLLER_HPP_

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
#include "geometry_msgs/msg/pose.hpp"

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace fr3_custom_controllers {
  using Vector8d = Eigen::Vector<double, 8>;
  using Vector6d = Eigen::Matrix<double, 6, 1>;

  class OperationalSpaceController : public controller_interface::ControllerInterface 
  {

    public:
      OperationalSpaceController();

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

      void osc(
        const Vector8d& q,
        const Vector8d& qd,
        const geometry_msgs::msg::Pose& pose_des,
        Vector8d& tau
      );

        
    private:

      // rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr joint_command_subscriber;
      // realtime_tools::RealtimeBuffer<std::shared_ptr<std_msgs::msg::Float64MultiArray>> joint_msg_external_ptr;
      // std::atomic<bool> new_msg_ = false;
      // std::shared_ptr<std_msgs::msg::Float64MultiArray> joint_msg_;
      
      
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
      std::string ee_name;
      std::string base_name;
      
      pinocchio::Model model;
      pinocchio::Data data;
      pinocchio::FrameIndex ee_frame_idx;
      pinocchio::FrameIndex base_frame_index;
      
      // Task space gains
      Eigen::Vector<double, 6> kp;
      Eigen::Vector<double, 6> kd;
      
      // Inputs to osc algorithm
      Vector8d q; // Current joint angles
      Vector8d qd; // Current joint velocities
      geometry_msgs::msg::Pose pose_des; // Desired end effector pose
      Vector8d tau0; // null space control torque (all zeros right now)
      
      // Used inside the algorithm
      Eigen::MatrixXd M; // Inertia matrix
      Eigen::MatrixXd Minv;
      Eigen::VectorXd b; // Nonlinear effects matrix = C(q,qd)*qd + g(q)
      Eigen::MatrixXd J; // EE jacobian
      Eigen::MatrixXd Jtrans;
      Eigen::MatrixXd JJtrans; // J * Jtrans
      Eigen::MatrixXd Jpseudo; // J pseudoinverse
      Eigen::MatrixXd Jdot; // dJ/dt
      Eigen::MatrixXd lambda; // Task-space inertia
      Eigen::MatrixXd Jdyncons; // Dynamically consistent inverse
      pinocchio::SE3 pose_cur; // Currend end effector pose
      Eigen::Vector3d trans_des;
      Eigen::Vector3d trans_err;
      Eigen::Quaterniond quat_des;
      Eigen::Quaterniond quat_err;
      Vector6d x_err; // cartesian position error (orientation and position) (used in PD xdd_cmd calc)
      Vector6d xd; // cartesian end effector frame velocity
      Vector6d xdd_cmd; // commanded cartesian acceleration
      Eigen::MatrixXd I; // 8x8 identity matrix
      Eigen::MatrixXd null_space_tau; // null space effort

      // Output (edited inplace)
      Vector8d tau = Vector8d::Zero();

      };

}

#endif