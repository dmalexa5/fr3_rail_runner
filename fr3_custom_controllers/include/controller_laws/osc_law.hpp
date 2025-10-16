#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/compute-all-terms.hpp>

void update(const Eigen::VectorXd &q,
            const Eigen::VectorXd &dq,
            Eigen::VectorXd &tau_out)
{
    // Assume: model, data, and all matrices are class members.
    pinocchio::forwardKinematics(model, data, q, dq);
    pinocchio::computeJointJacobians(model, data, q);
    pinocchio::centerOfMass(model, data, q, dq); // optional

    // Compute dynamics terms
    pinocchio::crba(model, data, q);       // M in data.M
    pinocchio::nonLinearEffects(model, data, q, dq); // C + g
    Eigen::MatrixXd M = data.M;
    Eigen::VectorXd Cg = data.nle;         // Coriolis + gravity

    // End-effector Jacobian
    Eigen::MatrixXd J = pinocchio::computeFrameJacobian(model, data, q, ee_frame_id,
                                                        pinocchio::ReferenceFrame::LOCAL);

    // Forward kinematics: frame placement
    const auto &oMf = pinocchio::updateFramePlacement(model, data, ee_frame_id);
    Eigen::Vector3d x = oMf.translation();   // position
    // (orientation e.g. from oMf.rotation())

    // Compute task-space inertia
    Eigen::MatrixXd JMJt = J * M.inverse() * J.transpose();
    Eigen::MatrixXd Lambda = JMJt.inverse();

    // Task-space bias forces
    Eigen::VectorXd mu_p = Lambda * J * M.inverse() * Cg;

    // Desired task-space accel (for simplicity, zero-vel, zero-acc)
    Eigen::VectorXd x_des(6), xdot_des(6), xddot_des(6);
    x_des.setZero();
    xdot_des.setZero();
    xddot_des.setZero();

    Eigen::VectorXd x_curr(6); // fill with position + orientation error
    Eigen::VectorXd xdot_curr(6); // compute from J * dq

    Eigen::MatrixXd Kp = 100 * Eigen::MatrixXd::Identity(6,6);
    Eigen::MatrixXd Kd = 20 * Eigen::MatrixXd::Identity(6,6);

    Eigen::VectorXd xddot_cmd =
        xddot_des + Kd * (xdot_des - xdot_curr) + Kp * (x_des - x_curr);

    // Compute task-space force
    Eigen::VectorXd F_task = Lambda * xddot_cmd + mu_p;

    // Finally: map to joint torques
    tau_out = J.transpose() * F_task;
}