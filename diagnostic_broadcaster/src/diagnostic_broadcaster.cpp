#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

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

  

}

namespace diagnostic_broadcaster
{

  DiagnosticBroadcaster::DiagnosticBroadcaster() {};

  

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_init()
  {
    joint_names_.clear();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  controller_interface::InterfaceConfiguration
  DiagnosticBroadcaster::command_interface_configuration() const
  {
    return controller_interface::InterfaceConfiguration{
        controller_interface::interface_configuration_type::NONE};
  }

  controller_interface::InterfaceConfiguration DiagnosticBroadcaster::state_interface_configuration() const
  {
    controller_interface::InterfaceConfiguration state_interfaces_config;

    if (get_joint_names().empty())
    {
      state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    }
    else
    {
      state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      for (size_t i = 0; i < joint_names_.size(); i++)
      {
        for (size_t j = 0; j < interface_names.size(); j++)
        {
          state_interfaces_config.names.push_back(joint_names_[i] + "/" + interface_names[j]);
        }
      }
    }

    return state_interfaces_config;
  }

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_configure(
      const rclcpp_lifecycle::State & /*previous_state*/)
  {
    try
    {
      diagnostic_publisher_ = get_node()->create_publisher<diagnostic_msgs::msg::Diagnostics>(
          diagnostic_broadcaster::DEFAULT_DIAGNOSTIC_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_publisher_ =
          std::make_unique<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::Diagnostics>>(
              diagnostic_publisher_);
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(
          get_node()->get_logger(), "Exception thrown during publisher creation at configure stage with message: %s\n",
          ex.what());
      return controller_interface::CallbackReturn::ERROR;
    }

    // Initialize diagnostic message
    realtime_publisher_->lock();
    realtime_publisher_->msg_.header.frame_id = "Diagnostics" /*params_.frame_id*/;
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
    for (size_t i = 0; i < interface_names.size(); i++)
    {
      if (_interface_name == interface_names[i])
        return true;
    }
    return false;
  }

  bool DiagnosticBroadcaster::init_joint_data()
  {

    temperature_interfaces_.clear();
    fault_interfaces_.clear();

    if (state_interfaces_.empty())
    {
      return false;
    }

    if(joint_names_.empty())
    {
      for(const auto& interface: state_interfaces_)
      {
        if(interface.get_interface_name() == "temperature")
        {
          joint_names_.push_back(interface.get_prefix_name());
        }
      }
    }
    joint_num_ = joint_names_.size();

    temperature_interfaces_.reserve(joint_num_);
    fault_interfaces_.reserve(joint_num_);

    
    for(size_t i = 0; i < state_interfaces_.size(); i++)
    {
      std::string joint_name_temp =  state_interfaces_[i].get_interface_name();

      if(joint_name_temp == "temperature")
        temperature_interfaces_.push_back(state_interfaces_[i]);
      else if(joint_name_temp == "fault")
        fault_interfaces_.push_back(state_interfaces_[i]);
    }

    return true;
  }

  void DiagnosticBroadcaster::init_realtime_publisher_msg()
  {

    auto &realtime_publisher_msg = realtime_publisher_->msg_;

    realtime_publisher_msg.joints = joint_names_;
    realtime_publisher_msg.temperature.resize(joint_num_, kUninitializedValue);
    realtime_publisher_msg.fault.resize(joint_num_, -1);
    // @note ADD NEW LINE FOR NEW INTERFACES (realtime_publisher_msg.<new>.resize(num_joints, kUninitializedValue);
  }


  //@note For individual joint peaking
  void DiagnosticBroadcaster::assign_joints(std::vector<std::string> assigned_state_interfaces)
  {
    for (size_t i = 0; i < assigned_state_interfaces.size(); i++)
    {
      bool hasFound = false;
      for (size_t j = 0; j < joint_names_.size(); j++)
      {
        if (joint_names_[j] == assigned_state_interfaces[i])
        {
          hasFound = true;
          break;
        }
      }
      if (hasFound == false)
        joint_names_.push_back(assigned_state_interfaces[i]);
    }
  }

  controller_interface::return_type DiagnosticBroadcaster::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (realtime_publisher_ && realtime_publisher_->trylock())
    {
      
      realtime_publisher_->msg_.header.stamp = time;

      if (temperature_interfaces_.size() != fault_interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Temperature and fault interfaces size mismatch!");
        return controller_interface::return_type::ERROR;
      }



      for(int i = 0; i < joint_num_; i++)
      {
        double temp_value_ =  temperature_interfaces_[i].get().get_value();
        int8_t fault_value_ = static_cast<int>(fault_interfaces_[i].get().get_value());
        
        realtime_publisher_->msg_.temperature[i] = temp_value_;
        realtime_publisher_->msg_.fault[i] = fault_value_;

        RCLCPP_DEBUG(
            get_node()->get_logger(),
            "Updated joint: %s, temperature: %f, fault: %d",
            joint_names_[i].c_str(), temp_value_, fault_value_);
      }
      
      // Publish the message
      realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diagnostic_broadcaster::DiagnosticBroadcaster, controller_interface::ControllerInterface)
