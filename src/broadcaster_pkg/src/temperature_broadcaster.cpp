#include "temperature_broadcaster/temperature_broadcaster.hpp"

namespace
{

constexpr auto DEFAULT_TEMPERATURE_TOPIC = "~/temperature";
constexpr auto DEFAULT_TF_TOPIC = "/tf";

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
  tf_publish_period_ =
    params_.tf.publish_rate == 0.0
      ? std::nullopt
      : std::optional{rclcpp::Duration::from_seconds(1.0 / params_.tf.publish_rate)};

  try
  {
    temperature_publisher_ = get_node()->create_publisher<temperature_msgs::msg::TemperatureBroadcast>(
      DEFAULT_TEMPERATURE_TOPIC, rclcpp::SystemDefaultsQoS());
    realtime_publisher_ =
      std::make_unique<realtime_tools::RealtimePublisher<temperature_msgs::msg::TemperatureBroadcast>>(
        temperature_publisher_);

    if (params_.tf.enable)
    {
      tf_publisher_ = get_node()->create_publisher<tf2_msgs::msg::TFMessage>(
        DEFAULT_TF_TOPIC, rclcpp::SystemDefaultsQoS());
      realtime_tf_publisher_ =
        std::make_unique<realtime_tools::RealtimePublisher<tf2_msgs::msg::TFMessage>>(
          tf_publisher_);
    }
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

  // Initialize tf message if tf publishing is enabled
  if (realtime_tf_publisher_)
  {
    realtime_tf_publisher_->lock();

    realtime_tf_publisher_->msg_.transforms.resize(1);
    auto & tf_transform = realtime_tf_publisher_->msg_.transforms.front();
    tf_transform.header.frame_id = params_.frame_id;
    if (params_.tf.child_frame_id.empty())
    {
      tf_transform.child_frame_id = params_.temperature_name;
    }
    else
    {
      tf_transform.child_frame_id = params_.tf.child_frame_id;
    }

    realtime_tf_publisher_->unlock();
  }

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
  geometry_msgs::msg::Pose temperature;
  temperature_sensor_->get_values_as_message(temperature);

  if (realtime_publisher_ && realtime_publisher_->trylock())
  {
    realtime_publisher_->msg_.header.stamp = time;
    realtime_publisher_->msg_.temperature = temperature;
    realtime_publisher_->unlockAndPublish();
  }

  if (realtime_tf_publisher_ && realtime_tf_publisher_->trylock())
  {
    bool do_publish = false;
    // rlcpp::Time comparisons throw if clock types are not the same
    if (tf_last_publish_time_.get_clock_type() != time.get_clock_type())
    {
      do_publish = true;
    }
    else if (!tf_publish_period_ || (tf_last_publish_time_ + *tf_publish_period_ <= time))
    {
      do_publish = true;
    }

    if (do_publish)
    {
      auto & tf_transform = realtime_tf_publisher_->msg_.transforms[0];
      tf_transform.header.stamp = time;

      tf_transform.transform.translation.x = temperature.position.x;
      tf_transform.transform.translation.y = temperature.position.y;
      tf_transform.transform.translation.z = temperature.position.z;

      tf_transform.transform.rotation.x = temperature.orientation.x;
      tf_transform.transform.rotation.y = temperature.orientation.y;
      tf_transform.transform.rotation.z = temperature.orientation.z;
      tf_transform.transform.rotation.w = temperature.orientation.w;

      realtime_tf_publisher_->unlockAndPublish();

      tf_last_publish_time_ = time;
    }
    else
    {
      realtime_tf_publisher_->unlock();
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace temperature_broadcaster

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(temperature_broadcaster::TemperatureBroadcaster, controller_interface::ControllerInterface)