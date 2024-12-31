#include "temperature_broadcaster/temperature_broadcaster.hpp"

namespace
{

constexpr auto DEFAULT_TEMPERATURE_TOPIC = "~/temperature";


}  // namespace

namespace temperature_broadcaster
{

controller_interface::InterfaceConfiguration TemperatureBroadcaster::command_interface_configuration()
  const
{
  controller_interface::InterfaceConfiguration command_interfaces_config;
  command_interfaces_config.type = controller_interface::interface_configuration_type::NONE;
  return command_interfaces_config;
}

controller_interface::InterfaceConfiguration TemperatureBroadcaster::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration state_interfaces_config;
  state_interfaces_config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  state_interfaces_config.names = temperature_sensor_->get_state_interface_names();

  return state_interfaces_config;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_init()
{
  try
  {
    param_listener_ = std::make_shared<ParamListener>(get_node());
    params_ = param_listener_->get_params();
  }
  catch (const std::exception & ex)
  {
    fprintf(stderr, "Exception thrown during init stage with message: %s\n", ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  params_ = param_listener_->get_params();

  temperature_sensor_ = std::make_unique<semantic_components::PoseSensor>(params_.temperature_name);

  try
  {
    temperature_publisher_ = get_node()->create_publisher<temperature_msgs::msg::TemperatureBroadcast>(
      DEFAULT_TEMPERATURE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<temperature_msgs::msg::TemperatureBroadcast>>(
        temperature_publisher_);
  }
  catch (const std::exception & ex)
  {
    fprintf(
      stderr, "Exception thrown during publisher creation at configure stage with message: %s\n",
      ex.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize temperature message
  realtime_publisher_->lock();
  realtime_publisher_->msg_.header.frame_id = params_.frame_id;
  realtime_publisher_->unlock();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  temperature_sensor_->assign_loaned_state_interfaces(state_interfaces_);
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn TemperatureBroadcaster::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  temperature_sensor_->release_interfaces();
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type TemperatureBroadcaster::update(
  const rclcpp::Time & time, const rclcpp::Duration & /*period*/)
{
  temperature_msgs::msg::TemperatureBroadcast temperature;
  temperature_sensor_->get_values_as_message(temperature);

  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.temperature = temperature;
    realtime_publisher_->unlockAndPublish();
  }

  return controller_interface::return_type::OK;
}

}  // namespace temperature_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(temperature_broadcaster::TemperatureBroadcaster, controller_interface::ControllerInterface)