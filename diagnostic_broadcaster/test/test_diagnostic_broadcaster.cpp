#include "test_diagnostic_broadcaster.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using hardware_interface::LoanedStateInterface;


void DiagnosticBroadcasterTest::SetUp() 
{  
  diagnostic_broadcaster_ = std::make_unique<FriendDiagnosticBroadcasterTest>(); 
}

void DiagnosticBroadcasterTest::TearDown() { diagnostic_broadcaster_.reset(NULL); }

void DiagnosticBroadcasterTest::SetUpDiagnosticBroadcaster()
{
    auto node = rclcpp::Node::make_shared("test_node");

    // Jawne wartości domyślne
    node->declare_parameter<std::vector<std::string>>(
        "diagnostic_broadcaster.joint_names", std::vector<std::string>{});
    node->declare_parameter<std::vector<std::string>>(
        "diagnostic_broadcaster.interface_names", std::vector<std::string>{
          "temperature",
          "fault", 
          "motor_effort", 
          "motor_position", 
          "motor_desired_position", 
          "motor_position_error", 
          "motor_velocity", 
          "motor_desired_velocity", 
          "motor_velocity_error", 
          "motor_desired_torque", 
          "power", 
          "current", 
          "voltage"});
    node->declare_parameter<double>(
        "diagnostic_broadcaster.interface_params.__map_interface_names.update_threshold", 0.1);

    diagnostic_broadcaster_->init("test_diagnostic_broadcaster", "", rclcpp::NodeOptions());

    std::vector<LoanedStateInterface> state_interfaces;
    state_interfaces.emplace_back(temperature_interface_1);
    state_interfaces.emplace_back(fault_interface_1);
    state_interfaces.emplace_back(motor_effort_interface_1);
    state_interfaces.emplace_back(mposition_interface_1);
    state_interfaces.emplace_back(mdesired_position_interface_1);
    state_interfaces.emplace_back(mposition_error_interface_1);
    state_interfaces.emplace_back(mvelocity_interface_1);
    state_interfaces.emplace_back(mdesired_velocity_interface_1);
    state_interfaces.emplace_back(mvelocity_error_interface_1);
    state_interfaces.emplace_back(mdesired_torque_interface_1);
    state_interfaces.emplace_back(power_interface_1);
    state_interfaces.emplace_back(current_interface_1);
    state_interfaces.emplace_back(voltage_interface_1);

    state_interfaces.emplace_back(temperature_interface_2);
    state_interfaces.emplace_back(fault_interface_2);
    state_interfaces.emplace_back(motor_effort_interface_2);


    state_interfaces.emplace_back(pressure_interface_3);
    
    EXPECT_TRUE(diagnostic_broadcaster_->get_joint_names().empty());

    diagnostic_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
}


TEST_F(DiagnosticBroadcasterTest, Configure_Success)
{
  //ZROBIONY DO DiagnosticBroadcasterTest
  SetUpDiagnosticBroadcaster();

  // Configure controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command interface configuration
  const auto command_interface_conf = diagnostic_broadcaster_->command_interface_configuration();
  EXPECT_EQ(command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
  EXPECT_TRUE(command_interface_conf.names.empty());

  // Verify state interface configuration
  const auto state_interface_conf = diagnostic_broadcaster_->state_interface_configuration();
  EXPECT_EQ(
    state_interface_conf.type, controller_interface::interface_configuration_type::ALL);
  

  ASSERT_EQ(diagnostic_broadcaster_->state_interfaces_.size(), 17lu);
}
TEST_F(DiagnosticBroadcasterTest, Activate_Success)
{
  SetUpDiagnosticBroadcaster();
  
  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command and state interface configuration
  {
    const auto command_interface_conf = diagnostic_broadcaster_->command_interface_configuration();
    EXPECT_EQ(
      command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
    EXPECT_TRUE(command_interface_conf.names.empty());
    const auto state_interface_conf = diagnostic_broadcaster_->state_interface_configuration();
    EXPECT_EQ(
      state_interface_conf.type, controller_interface::interface_configuration_type::ALL);
    ASSERT_EQ(diagnostic_broadcaster_->state_interfaces_.size(), 17lu);
  }

  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Deactivate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_deactivate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command and state interface configuration
  {
    const auto command_interface_conf = diagnostic_broadcaster_->command_interface_configuration();
    EXPECT_EQ(
      command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
    EXPECT_TRUE(command_interface_conf.names.empty());

    const auto state_interface_conf = diagnostic_broadcaster_->state_interface_configuration();
    EXPECT_EQ(
      state_interface_conf.type, controller_interface::interface_configuration_type::ALL);
    ASSERT_EQ(diagnostic_broadcaster_->state_interfaces_.size(), 0);
  }
}

TEST_F(DiagnosticBroadcasterTest, Update_Success)
{
  SetUpDiagnosticBroadcaster();

  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  ASSERT_EQ(
    diagnostic_broadcaster_->update(rclcpp::Time{0}, rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
}

TEST_F(DiagnosticBroadcasterTest, PublishSuccess)
{
  SetUpDiagnosticBroadcaster();

  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Subscribe to diagnostic topic
  diagnostic_msgs::msg::Diagnostics diagnostic_msg;
  subscribe_and_get_message("/test_diagnostic_broadcaster/diagnostics", diagnostic_msg);

  // Verify content of diagnostic message
  EXPECT_EQ(diagnostic_msg.names.size(), 2lu);

  EXPECT_EQ(diagnostic_msg.header.frame_id, "Diagnostics");
  EXPECT_EQ(diagnostic_msg.names[0], "test_joint1");
  EXPECT_EQ(diagnostic_msg.names[1], "test_joint2");

  EXPECT_EQ(diagnostic_msg.temperature[0], example_values_[0]);
  EXPECT_EQ(diagnostic_msg.temperature[1], example_values_[2]);
  
  int8_t expected = 1;
  EXPECT_EQ(diagnostic_msg.fault[0], expected);
  EXPECT_EQ(diagnostic_msg.fault[1], expected);

  EXPECT_EQ(diagnostic_msg.motor_effort[0], example_values_[4]);
  EXPECT_EQ(diagnostic_msg.motor_effort[1], example_values_[4]);

  EXPECT_EQ(diagnostic_msg.motor_position[0], example_values_[6]);
  EXPECT_EQ(diagnostic_msg.motor_position[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_desired_position[0], example_values_[8]);
  EXPECT_EQ(diagnostic_msg.motor_desired_position[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_position_error[0], example_values_[0]);
  EXPECT_EQ(diagnostic_msg.motor_position_error[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_velocity[0], example_values_[2]);
  EXPECT_EQ(diagnostic_msg.motor_velocity[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_desired_velocity[0], example_values_[4]);
  EXPECT_EQ(diagnostic_msg.motor_desired_velocity[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_velocity_error[0], example_values_[6]);
  EXPECT_EQ(diagnostic_msg.motor_velocity_error[1], 0.0);

  EXPECT_EQ(diagnostic_msg.motor_desired_torque[0], example_values_[8]);
  EXPECT_EQ(diagnostic_msg.motor_desired_torque[1], 0.0);

  EXPECT_EQ(diagnostic_msg.power[0], example_values_[0]);
  EXPECT_EQ(diagnostic_msg.power[1], 0.0);

  EXPECT_EQ(diagnostic_msg.current[0], example_values_[2]);
  EXPECT_EQ(diagnostic_msg.current[1], 0.0);

  EXPECT_EQ(diagnostic_msg.voltage[0], example_values_[4]);
  EXPECT_EQ(diagnostic_msg.voltage[1], 0.0);

}

TEST_F(DiagnosticBroadcasterTest, ThresholdTest)
{
  SetUpDiagnosticBroadcaster();

  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);


  diagnostic_msgs::msg::Diagnostics diagnostic_msg;


  // Value greater than threshold
  example_values_[0] = 31.0f; 

  subscribe_and_get_message("/test_diagnostic_broadcaster/diagnostics", diagnostic_msg);

  EXPECT_EQ(diagnostic_msg.temperature[0], 31.0f);

  //Value smaller than threshold
  example_values_[0] = 31.05f; 

  subscribe_and_get_message("/test_diagnostic_broadcaster/diagnostics", diagnostic_msg);

  EXPECT_EQ(diagnostic_msg.temperature[0], 31.0f);
}


int main(int argc, char * argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}