#pragma once

#include <vector>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <Eigen/Dense>

#include "geometry_msgs/msg/pose.hpp"
#include "pinocchio/spatial/se3.hpp"


namespace fr3_custom_controllers {

  inline std::string eigen_str(const Eigen::MatrixXd& mat) {
    std::stringstream ss;
    ss << std::fixed << std::setprecision(3);

    for (int i = 0; i < mat.rows(); ++i) {
        ss << "[ ";
        for (int j = 0; j < mat.cols(); ++j) {
            ss << std::setw(8) << mat(i, j);
            if (j < mat.cols() - 1) ss << ", ";
        }
        ss << " ]";
        if (i < mat.rows() - 1) ss << "\n";
    }

    return ss.str();

  }

  inline std::vector<std::vector<double>> eigen_to_vec(const Eigen::MatrixXd& mat) {
    std::vector<std::vector<double>> vec(mat.rows(), std::vector<double>(mat.cols()));

    for (int i = 0; i < mat.rows(); ++i) {
        for (int j = 0; j < mat.cols(); ++j) {
            double value = mat(i, j);
            vec[i][j] = std::round(value * 1000.0) / 1000.0;
        }
    }

    return vec;
  }
  
  inline pinocchio::SE3 pose_to_SE3(const geometry_msgs::msg::Pose &pose_msg) {

    Eigen::Vector3d translation(
        pose_msg.position.x,
        pose_msg.position.y,
        pose_msg.position.z);

    Eigen::Quaterniond quat(
        pose_msg.orientation.w,
        pose_msg.orientation.x,
        pose_msg.orientation.y,
        pose_msg.orientation.z);
    
    quat.normalize();

    pinocchio::SE3 se3_pose(quat.toRotationMatrix(), translation);

    return se3_pose;
  }

  inline std::string loadURDF(const std::string& path) {
    std::ifstream file(path);
    std::stringstream buffer;
    buffer << file.rdbuf();
    return buffer.str();
  }

  inline void computePoseError(
    const pinocchio::SE3& pose_des,
    const pinocchio::SE3& pose_cur,
    Eigen::Matrix<double,6,1>& x_err)
  {
    const pinocchio::SE3 M_err = pose_cur.inverse() * pose_des;
    const Eigen::Matrix3d& R = M_err.rotation();
    const Eigen::Vector3d& t = M_err.translation();

    // Small-angle safe approximation
    Eigen::Vector3d omega;
    double theta = std::acos(std::clamp((R.trace() - 1.0) / 2.0, -1.0, 1.0));
    if (theta < 1e-6)
      omega.setZero();
    else
      omega = (theta / (2.0 * std::sin(theta))) * Eigen::Vector3d(R(2,1) - R(1,2),
                                                                  R(0,2) - R(2,0),
                                                                  R(1,0) - R(0,1));
    x_err.head<3>() = omega;
    x_err.tail<3>() = t;
  }
}