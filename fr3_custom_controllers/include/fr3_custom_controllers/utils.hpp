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
}