#ifndef TEST_DIAGNOSTIC_BROADCASTER_HPP_
#define TEST_DIAGNOSTIC_BROADCASTER_HPP_

#include <gmock/gmock.h>

#include <array>
#include <memory>
#include <string>
#include <limits>
#include "rclcpp/executors.hpp"

#include "diagnostic_broadcaster/diagnostic_broadcaster.hpp"

using diagnostic_broadcaster::DiagnosticBroadcaster;

class FriendDiagnosticBroadcasterTest : public DiagnosticBroadcaster
{
  FRIEND_TEST(DiagnosticBroadcasterTest, Configure_Success);
  FRIEND_TEST(DiagnosticBroadcasterTest, Activate_Success);
};

class DiagnosticBroadcasterTest : public ::testing::Test
{

public:
  void SetUp();
  void TearDown();
  void SetUpDiagnosticBroadcaster();
  void CheckJointsName(std::vector<std::string> actual_joint_names, std::vector<std::string> wanted_joint_names);
  const std::vector<hardware_interface::LoanedStateInterface>  &get_state_interfaces() const;
  
protected:
  std::array<double, 10> example_values_ = {0.0, 0.1, 20.0, 20.1, 30.0, 30.1, 40.0, 40.1, 50.0, 50.1};
  
  double fault = 1.0;
  
  //joint1
  hardware_interface::StateInterface temperature_interface_1{"test_joint1", "temperature", &example_values_[0]};
  hardware_interface::StateInterface fault_interface_1{"test_joint1", "fault", &fault};
  hardware_interface::StateInterface motor_effort_interface_1{"test_joint1", "motor_effort", &example_values_[4]};
  hardware_interface::StateInterface mposition_interface_1{"test_joint1", "motor_position", &example_values_[6]};
  hardware_interface::StateInterface mdesired_position_interface_1{"test_joint1", "motor_desired_position", &example_values_[8]};
  hardware_interface::StateInterface mposition_error_interface_1{"test_joint1", "motor_position_error", &example_values_[0]};
  hardware_interface::StateInterface mvelocity_interface_1{"test_joint1", "motor_velocity", &example_values_[2]};
  hardware_interface::StateInterface mdesired_velocity_interface_1{"test_joint1", "motor_desired_velocity", &example_values_[4]};
  hardware_interface::StateInterface mvelocity_error_interface_1{"test_joint1", "motor_velocity_error", &example_values_[6]};
  hardware_interface::StateInterface mdesired_torque_interface_1{"test_joint1", "motor_desired_torque", &example_values_[8]};
  hardware_interface::StateInterface power_interface_1{"test_joint1", "power", &example_values_[0]};
  hardware_interface::StateInterface current_interface_1{"test_joint1", "current", &example_values_[2]};
  hardware_interface::StateInterface voltage_interface_1{"test_joint1", "voltage", &example_values_[4]};
  hardware_interface::StateInterface pressure_interface_1{"test_joint1", "pressure", &example_values_[4]};

  //joint2
  hardware_interface::StateInterface temperature_interface_2{"test_joint2", "temperature", &example_values_[2]};
  hardware_interface::StateInterface fault_interface_2{"test_joint2", "fault", &fault};
  hardware_interface::StateInterface motor_effort_interface_2{"test_joint2", "motor_effort", &example_values_[4]};

  //joint3
  hardware_interface::StateInterface pressure_interface_3{"test_joint3", "pressure", &example_values_[4]};


  std::unique_ptr<FriendDiagnosticBroadcasterTest> diagnostic_broadcaster_;
  
  template <typename T>
  void subscribe_and_get_message(const std::string &topic, T &msg);
  
  
  std::string printJointNames(std::vector<std::string> joint_names);

};

template <typename T>
void DiagnosticBroadcasterTest::subscribe_and_get_message(const std::string &topic, T &msg)
{
  // Create node for subscribing
  rclcpp::Node node{"test_subscription_node"};
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node.get_node_base_interface());

  // Create subscription
  typename T::SharedPtr received_msg;
  const auto msg_callback = [&](const typename T::SharedPtr sub_msg)
  { received_msg = sub_msg; };
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

    diagnostic_broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01));

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

void DiagnosticBroadcasterTest::CheckJointsName(std::vector<std::string> actual_joint_names, std::vector<std::string> wanted_joint_names)
{
  for (size_t i = 0; i < actual_joint_names.size(); i++)
  {
    EXPECT_EQ(actual_joint_names[i], wanted_joint_names[i]);
  }
}

#endif