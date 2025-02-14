#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp/qos_event.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rcpputils/split.hpp"
#include "rcutils/logging_macros.h"
#include "std_msgs/msg/header.hpp"

namespace
{

constexpr auto DEFAULT_DIAGNOSTIC_TOPIC = "~/diagnostics";

}  

namespace diagnostic_broadcaster
{

DiagnosticBroadcaster::DiagnosticBroadcaster(){};
const auto kUninitializedValue = std::numeric_limits<double>::quiet_NaN();


controller_interface::CallbackReturn DiagnosticBroadcaster::on_init()
{
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
DiagnosticBroadcaster::command_interface_configuration() const
{
  return controller_interface::InterfaceConfiguration{
    controller_interface::interface_configuration_type::NONE};
}

controller_interface::InterfaceConfiguration DiagnosticBroadcaster::state_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;

  state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;

  return state_interfaces_config;
}



controller_interface::CallbackReturn DiagnosticBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  try
  {
    diagnostic_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::Diagnostics>(
      DEFAULT_DIAGNOSTIC_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::Diagnostics>>(
        diagnostic_publisher_);
  }
  catch (const std::exception & ex)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message: %s\n",
      ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize diagnostic message
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = "Diagnostics"/*params_.frame_id*/;
  realtime_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiagnosticBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!init_joint_data())
  {
    RCLCPP_ERROR(
      get_node()->get_logger(), "None of requested interfaces exist. Controller will not run.");
    return CallbackReturn::ERROR;
  }

  init_realtime_publisher_msg();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn DiagnosticBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  joint_names_.clear();
  return controller_interface::CallbackReturn::SUCCESS;
}


bool DiagnosticBroadcaster::has_any_key(std::string _interface_name)
{
  for(size_t i = 0; i < interface_names.size(); i++)
  {
    if(_interface_name == interface_names[i]) return true;
  }
  return false;
}

bool DiagnosticBroadcaster::init_joint_data()
{
  joint_state_interfaces_.clear();

  if (state_interfaces_.empty())
  {
    return false;
  }

  for (auto si = state_interfaces_.rbegin(); si != state_interfaces_.rend(); si++)
  {
    if (has_any_key(si->get_interface_name()))
    {
      joint_state_interfaces_.push_back(std::ref(*si));
    }
  }

  return true;
}

void DiagnosticBroadcaster::init_realtime_publisher_msg()
{
  const size_t num_joints = joint_names_.size();

  
  auto & realtime_publisher_msg = realtime_publisher_->msg_;
  realtime_publisher_msg.joints = joint_names_;
  realtime_publisher_msg.temperature.resize(num_joints, kUninitializedValue);
  // @note ADD NEW LINE FOR NEW INTERFACES (realtime_publisher_msg.<new>.resize(num_joints, kUninitializedValue);
}

controller_interface::return_type DiagnosticBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    // Set the timestamp
    realtime_publisher_->msg_.header.stamp = time;

    // Clear existing data in the message
    realtime_publisher_->msg_.joints.clear();
    realtime_publisher_->msg_.temperature.clear();

    // Iterate through state_interfaces_
    for (auto si = joint_state_interfaces_.rbegin(); si != joint_state_interfaces_.rend(); si++)
    {
      const std::string joint_name = si->get().get_prefix_name();  // Dostęp przez get()
      const std::string interface_name = si->get().get_interface_name();
      double value = si->get().get_value();  // Pobranie wartości
    
      // Add the value to the message
      realtime_publisher_->msg_.joints.push_back(joint_name);
      realtime_publisher_->msg_.temperature.push_back(value);

      RCLCPP_DEBUG(
        get_node()->get_logger(),
        "Updated joint: %s, interface: %s, value: %f",
        joint_name.c_str(), interface_name.c_str(), value);
    }


    // Publish the message
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}



}  // namespace diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diagnostic_broadcaster::DiagnosticBroadcaster, controller_interface::ControllerInterface)