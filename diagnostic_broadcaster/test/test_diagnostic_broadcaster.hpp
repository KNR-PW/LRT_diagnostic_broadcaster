#ifndef TEST_DIAGNOSTIC_BROADCASTER_HPP_
#define TEST_DIAGNOSTIC_BROADCASTER_HPP_

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>

#include "rclcpp/executors.hpp"

#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

using diagnostic_broadcaster::DiagnosticBroadcaster;

class DiagnosticBroadcasterTest : public ::testing::Test
{

public:
  void SetUp();
  void TearDown();
  void SetUpPoseBroadcaster();

protected:
    const std::string joint_name_ = "test_pose";

    std::array<double, 7> temperature_values_ = {
    {30, 50, 20, 25.56034, 30.9999, 123, 60}};
    
    hardware_interface::StateInterface interface_1 {joint_name_ + "1", "temperature", &temperature_values_[0]};
    hardware_interface::StateInterface interface_2 {joint_name_ + "2", "temperature", &temperature_values_[1]};
    hardware_interface::StateInterface interface_3 {joint_name_ + "3", "temperature", &temperature_values_[2]};
    
    hardware_interface::StateInterface interface_4 {joint_name_ + "4", "temperature", &temperature_values_[3]};
    hardware_interface::StateInterface interface_4 {joint_name_ + "4", "pressure", &temperature_values_[3]};
    
    hardware_interface::StateInterface interface_5 {joint_name_ + "5", "pressure", &temperature_values_[4]};
    hardware_interface::StateInterface interface_6 {joint_name_ + "6", "position.x", &temperature_values_[5]};
    
    std::unique_ptr<DiagnosticBroadcaster> diagnostic_broadcaster_;

    template <typename T>
    void subscribe_and_get_message(const std::string & topic, T & msg);

}

template <typename T>
void PoseBroadcasterTest::subscribe_and_get_message(const std::string & topic, T & msg)
{
  // Create node for subscribing
  rclcpp::Node node{"test_subscription_node"};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());

  // Create subscription
  typename T::SharedPtr received_msg;
  const auto msg_callback = [&](const typename T::SharedPtr sub_msg) { received_msg = sub_msg; };
  const auto subscription = node.create_subscription<T>(topic, 10, msg_callback);

  // Update controller and spin until a message is received
  // Since update doesn't guarantee a published message, republish until received
  constexpr size_t max_sub_check_loop_count = 5;
  for (size_t i = 0; !received_msg; ++i)
  {
    if (i >= max_sub_check_loop_count)
    {
      throw std::runtime_error("Failed to receive message on topic: " + topic);
    }

    pose_broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01));

    const auto timeout = std::chrono::milliseconds{5};
    const auto until = node.get_clock()->now() + timeout;
    while (!received_msg && node.get_clock()->now() < until)
    {
      executor.spin_some();
      std::this_thread::sleep_for(std::chrono::microseconds{10});
    }
  }

  msg = *received_msg;
}