#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

#include <stddef.h>
#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <cmath>

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


namespace diagnostic_broadcaster
{

  DiagnosticBroadcaster::DiagnosticBroadcaster() {};

  controller_interface::CallbackReturn DiagnosticBroadcaster::on_init()
  {
    try
    {
      param_listener_ = std::make_shared<ParamListener>(get_node());
      params_ = param_listener_->get_params();
    }
    catch (const std::exception & e)
    {
      fprintf(stderr, "Exception thrown during init stage with message: %s \n", e.what());
      return CallbackReturn::ERROR;
    }

    previous_temp_val_.clear();
    previous_meffort_val_.clear();

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

    if (params_.joint_names.empty())
    {
      state_interfaces_config.type = controller_interface::interface_configuration_type::ALL;
    }
    else
    {
      state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
      for (size_t i = 0; i < params_.joint_names.size(); i++)
      {
        for (size_t j = 0; j < params_.interface_names.size(); j++)
        {
          state_interfaces_config.names.push_back(params_.joint_names[i] + "/" + params_.interface_names[j]);
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
    release_interfaces();
    return controller_interface::CallbackReturn::SUCCESS;
  }

  bool DiagnosticBroadcaster::has_a_key(const std::string &interface_name)
  {
    for (const auto & name : params_.interface_names)
    {
      if (interface_name == name)
        return true;
    }
    return false;
  }
  
  bool DiagnosticBroadcaster::init_joint_data()
  {
    joint_names_.clear();

    temperature_interfaces_.clear();
    fault_interfaces_.clear();
    meffort_interfaces_.clear();

    if (state_interfaces_.empty())
    {
      return false;
    }

    if(joint_names_.empty())
    {
      for(const auto& interface: state_interfaces_)
      {
        if(has_a_key(interface.get_interface_name()) && 
          std::find(joint_names_.begin(), joint_names_.end(), 
          interface.get_prefix_name()) == joint_names_.end())
        {
          joint_names_.push_back(interface.get_prefix_name());
        }
      }
    }

    joint_num_ = joint_names_.size();

    temperature_interfaces_.reserve(joint_num_);
    fault_interfaces_.reserve(joint_num_);
    meffort_interfaces_.reserve(joint_num_);

    for (const auto & joint_name : joint_names_)
    {
      for (auto & iface : state_interfaces_)
      {
        if (iface.get_prefix_name() == joint_name)
        {
          if (iface.get_interface_name() == "temperature")
            temperature_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "fault")
            fault_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_effort")
            meffort_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_position")
            mposition_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_desired_position")
            mdesired_position_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_position_error")
            mposition_error_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_velocity")
            mvelocity_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_desired_velocity")
            mdesired_velocity_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_velocity_error")
            mvelocity_error_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "motor_desired_torque")
            mdesired_torque_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "power")
            power_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "current")
            current_interfaces_.push_back(iface);
          else if (iface.get_interface_name() == "voltage")
            voltage_interfaces_.push_back(iface);
            
        }
     }
    }

    return true;
  }
  
  void DiagnosticBroadcaster::init_realtime_publisher_msg()
  {
    auto &realtime_publisher_msg = realtime_publisher_->msg_;
    
    realtime_publisher_msg.names = joint_names_;
    
    realtime_publisher_msg.temperature.resize(joint_num_, k_uninitialized_value_);
    previous_temp_val_.resize(joint_num_, k_uninitialized_value_);
    
    realtime_publisher_msg.fault.resize(joint_num_, -1);
    
    realtime_publisher_msg.motor_effort.resize(joint_num_, k_uninitialized_value_);
    previous_meffort_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_position.resize(joint_num_, k_uninitialized_value_);
    previous_mposition_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_desired_position.resize(joint_num_, k_uninitialized_value_);
    previous_mdesired_position_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_position_error.resize(joint_num_, k_uninitialized_value_);
    previous_mposition_error_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_velocity.resize(joint_num_, k_uninitialized_value_);
    previous_mvelocity_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_desired_velocity.resize(joint_num_, k_uninitialized_value_);
    previous_mdesired_velocity_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_velocity_error.resize(joint_num_, k_uninitialized_value_);
    previous_mvelocity_error_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.motor_desired_torque.resize(joint_num_, k_uninitialized_value_);
    previous_mdesired_torque_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.power.resize(joint_num_, k_uninitialized_value_);
    previous_power_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.current.resize(joint_num_, k_uninitialized_value_);
    previous_current_val_.resize(joint_num_, k_uninitialized_value_);

    realtime_publisher_msg.voltage.resize(joint_num_, k_uninitialized_value_);
    previous_voltage_val_.resize(joint_num_, k_uninitialized_value_);


    // @note ADD NEW LINE FOR NEW INTERFACES (realtime_publisher_msg.<new>.resize(num_joints, k_uninitialized_value);
  }

  controller_interface::return_type DiagnosticBroadcaster::update(
      const rclcpp::Time &time, const rclcpp::Duration & /*period*/)
  {
    if (realtime_publisher_ && realtime_publisher_->trylock())
    {
      realtime_publisher_->msg_.header.stamp = time;

      if (temperature_interfaces_.size() != fault_interfaces_.size() && temperature_interfaces_.size() != meffort_interfaces_.size()) {
        RCLCPP_ERROR(get_node()->get_logger(), "Interfaces size mismatch!");
        return controller_interface::return_type::ERROR;
      }

      realtime_publisher_->msg_.names = joint_names_;

      double temperature_threshold = params_.interface_params.interface_names_map["temperature"].update_threshold;
      double motor_effort_treshold = params_.interface_params.interface_names_map["motor_effort"].update_threshold;
      double motor_position_threshold = params_.interface_params.interface_names_map["motor_position"].update_threshold;
      double motor_desired_position_threshold = params_.interface_params.interface_names_map["motor_desired_position"].update_threshold;
      double motor_position_error_threshold = params_.interface_params.interface_names_map["motor_position_error"].update_threshold;
      double motor_velocity_threshold = params_.interface_params.interface_names_map["motor_velocity"].update_threshold;
      double motor_desired_velocity_threshold = params_.interface_params.interface_names_map["motor_desired_velocity"].update_threshold;
      double motor_velocity_error_threshold = params_.interface_params.interface_names_map["motor_velocity_error"].update_threshold;
      double motor_desired_torque_threshold = params_.interface_params.interface_names_map["motor_desired_torque"].update_threshold;
      double power_threshold = params_.interface_params.interface_names_map["power"].update_threshold;
      double current_threshold = params_.interface_params.interface_names_map["current"].update_threshold;
      double voltage_threshold = params_.interface_params.interface_names_map["voltage"].update_threshold;


      for(int i = 0; i < joint_num_; i++)
      {
        double temp_value_ =  temperature_interfaces_[i].get().get_value();
        double meffort_value_ = meffort_interfaces_[i].get().get_value();

        int8_t fault_value_ = static_cast<int>(fault_interfaces_[i].get().get_value());

        double mposition_value_ = mposition_interfaces_[i].get().get_value();
        double mdesired_position_value_ = mdesired_position_interfaces_[i].get().get_value();
        double mposition_error_value_ = mposition_error_interfaces_[i].get().get_value();

        double mvelocity_value_ = mvelocity_interfaces_[i].get().get_value(); 
        double mdesired_velocity_value_ = mdesired_velocity_interfaces_[i].get().get_value();
        double mvelocity_error_value_ = mvelocity_error_interfaces_[i].get().get_value();

        double mdesired_torque_value_ = mdesired_torque_interfaces_[i].get().get_value();

        double power_value_ = power_interfaces_[i].get().get_value();
        double current_value_ = current_interfaces_[i].get().get_value();
        double voltage_value_ = voltage_interfaces_[i].get().get_value();

        if(std::isnan(previous_temp_val_[i]) || 
          std::abs(temp_value_ - previous_temp_val_[i]) > temperature_threshold)
        {
          previous_temp_val_[i] = temp_value_;
          realtime_publisher_->msg_.temperature[i] = temp_value_;
        }

        if(std::isnan(previous_meffort_val_[i]) || 
          std::abs(meffort_value_ - previous_meffort_val_[i]) > motor_effort_treshold)
        {
          previous_meffort_val_[i] = meffort_value_;
          realtime_publisher_->msg_.motor_effort[i] = meffort_value_;
        }

        if(std::isnan(previous_mposition_val_[i]) || 
          std::abs(mposition_value_ - previous_mposition_val_[i]) > motor_position_threshold)
        {
          previous_mposition_val_[i] = mposition_value_;
          realtime_publisher_->msg_.motor_position[i] = mposition_value_;
        }

        if(std::isnan(previous_mdesired_position_val_[i]) || 
          std::abs(mdesired_position_value_ - previous_mdesired_position_val_[i]) > motor_desired_position_threshold)
        {
          previous_mdesired_position_val_[i] = mdesired_position_value_;
          realtime_publisher_->msg_.motor_desired_position[i] = mdesired_position_value_;
        }

        if(std::isnan(previous_mposition_error_val_[i]) || 
          std::abs(mposition_error_value_ - previous_mposition_error_val_[i]) > motor_position_error_threshold)
        {
          previous_mposition_error_val_[i] = mposition_error_value_;
          realtime_publisher_->msg_.motor_position_error[i] = mposition_error_value_;
        }

        if(std::isnan(previous_mvelocity_val_[i]) || 
          std::abs(mvelocity_value_ - previous_mvelocity_val_[i]) > motor_velocity_threshold)
        {
          previous_mvelocity_val_[i] = mvelocity_value_;
          realtime_publisher_->msg_.motor_velocity[i] = mvelocity_value_;
        }

        if(std::isnan(previous_mdesired_velocity_val_[i]) || 
          std::abs(mdesired_velocity_value_ - previous_mdesired_velocity_val_[i]) > motor_desired_velocity_threshold)
        {
          previous_mdesired_velocity_val_[i] = mdesired_velocity_value_;
          realtime_publisher_->msg_.motor_desired_velocity[i] = mdesired_velocity_value_;
        }

        if(std::isnan(previous_mvelocity_error_val_[i]) || 
          std::abs(mvelocity_error_value_ - previous_mvelocity_error_val_[i]) > motor_velocity_error_threshold)
        {
          previous_mvelocity_error_val_[i] = mvelocity_error_value_;
          realtime_publisher_->msg_.motor_velocity_error[i] = mvelocity_error_value_;
        }

        if(std::isnan(previous_mdesired_torque_val_[i]) || 
          std::abs(mdesired_torque_value_ - previous_mdesired_torque_val_[i]) > motor_desired_torque_threshold)
        {
          previous_mdesired_torque_val_[i] = mdesired_torque_value_;
          realtime_publisher_->msg_.motor_desired_torque[i] = mdesired_torque_value_;
        }

        if(std::isnan(previous_power_val_[i]) || 
          std::abs(power_value_ - previous_power_val_[i]) > power_threshold)
        {
          previous_power_val_[i] = power_value_;
          realtime_publisher_->msg_.power[i] = power_value_;
        }

        if(std::isnan(previous_current_val_[i]) || 
          std::abs(current_value_ - previous_current_val_[i]) > current_threshold)
        {
          previous_current_val_[i] = current_value_;
          realtime_publisher_->msg_.current[i] = current_value_;
        }

        if(std::isnan(previous_voltage_val_[i]) || 
          std::abs(voltage_value_ - previous_voltage_val_[i]) > voltage_threshold)
        {
          previous_voltage_val_[i] = voltage_value_;
          realtime_publisher_->msg_.voltage[i] = voltage_value_;
        }

        realtime_publisher_->msg_.fault[i] = fault_value_;

        // @note ADD NEW IF FOR NEW INTERFACES
      }
      
      // Publish the message
      realtime_publisher_->unlockAndPublish();
    }

    return controller_interface::return_type::OK;
  }

} // namespace diagnostic_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(diagnostic_broadcaster::DiagnosticBroadcaster, controller_interface::ControllerInterface)
