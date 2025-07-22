#include "test_diagnostic_broadcaster.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using hardware_interface::LoanedStateInterface;

void DiagnosticBroadcasterTest::SetUp() 
{   
  diagnostic_broadcaster_ = std::make_unique<DiagnosticBroadcaster>(); 
  diagnostic_broadcaster_->assign_joints({joint_name_ + "1", joint_name_ + "2", joint_name_ + "3", joint_name_ + "4", joint_name_ + "5"});
}

void DiagnosticBroadcasterTest::TearDown() { diagnostic_broadcaster_.reset(NULL); }

void DiagnosticBroadcasterTest::SetUpDiagnosticBroadcaster()
{

  ASSERT_EQ(
    diagnostic_broadcaster_->init("test_diagnostic_broadcaster"), controller_interface::return_type::OK);


  std::vector<LoanedStateInterface> state_interfaces;

  state_interfaces.emplace_back(interface_1);
  state_interfaces.emplace_back(interface_2);
  state_interfaces.emplace_back(interface_3);
  state_interfaces.emplace_back(interface_4);
  state_interfaces.emplace_back(interface_5);
  state_interfaces.emplace_back(interface_6);

  EXPECT_FALSE(diagnostic_broadcaster_->get_joint_names().empty());

  diagnostic_broadcaster_->assign_interfaces({}, std::move(state_interfaces));
}

TEST_F(DiagnosticBroadcasterTest, Configure_Success)
{
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
    state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
  ASSERT_EQ(state_interface_conf.names.size(), 10lu);
}
TEST_F(DiagnosticBroadcasterTest, Activate_Success)
{
  SetUpDiagnosticBroadcaster();

  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Verify command and state interface configuration
  {
    const auto command_interface_conf = diagnostic_broadcaster_->command_interface_configuration();
    EXPECT_EQ(
      command_interface_conf.type, controller_interface::interface_configuration_type::NONE);
    EXPECT_TRUE(command_interface_conf.names.empty());

    const auto state_interface_conf = diagnostic_broadcaster_->state_interface_configuration();
    EXPECT_EQ(
      state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    ASSERT_EQ(state_interface_conf.names.size(), 10lu);
  }

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
      state_interface_conf.type, controller_interface::interface_configuration_type::INDIVIDUAL);
    ASSERT_EQ(state_interface_conf.names.size(), 10lu);  // Should not change when deactivating
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
  EXPECT_EQ(diagnostic_msg.header.frame_id, "Diagnostics");
  EXPECT_EQ(diagnostic_msg.joints[0], joint_name_+"1");
  EXPECT_EQ(diagnostic_msg.joints[1], joint_name_+"2");
  EXPECT_EQ(diagnostic_msg.joints[2], joint_name_+"3");
  EXPECT_EQ(diagnostic_msg.joints[3], joint_name_+"4");
  EXPECT_EQ(diagnostic_msg.joints.size(), 5lu);

  EXPECT_EQ(diagnostic_msg.temperature[0], temperature_values_[0]);
  EXPECT_EQ(diagnostic_msg.temperature[1], temperature_values_[1]);
  EXPECT_EQ(diagnostic_msg.temperature[2], temperature_values_[2]);
  EXPECT_EQ(diagnostic_msg.temperature[3], temperature_values_[3]);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}