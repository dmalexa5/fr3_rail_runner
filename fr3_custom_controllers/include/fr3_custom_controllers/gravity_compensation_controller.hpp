#pragma once

#include <string>

#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterfaces::CallbackReturn;

namespace fr3_custom_controllers {

  class GravityCompensationController : public controller_interface::ControllerInterface {

    public:



    private:
      std::string arm_id_;

  }













}