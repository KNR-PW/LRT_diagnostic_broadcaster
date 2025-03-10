#include "test_diagnostic_broadcaster.hpp"

#include <cmath>
#include <limits>
#include <utility>
#include <vector>

using hardware_interface::LoanedStateInterface;

void PoseBroadcasterTest::SetUp() { diagnostic_broadcaster_ = std::make_unique<PoseBroadcaster>(); }

void PoseBroadcasterTest::TearDown() { diagnostic_broadcaster_.reset(NULL); }

void PoseBroadcasterTest::SetUpPoseBroadcaster()
{
  ASSERT_EQ(
    diagnostic_broadcaster_->on_init(), controller_interface::return_type::SUCCESS);

  std::vector<LoanedStateInterface> state_interfaces;

  state_interfaces.emplace_back(interface_1);
  state_interfaces.emplace_back(interface_2);
  state_interfaces.emplace_back(interface_3);
  state_interfaces.emplace_back(interface_4);
  state_interfaces.emplace_back(interface_5);
  state_interfaces.emplace_back(interface_6);


  diagnostic_broadcaster_->assign_interfaces(std::move(state_interfaces));
}

TEST_F(PoseBroadcasterTest, Configure_Success)
{
  SetUpPoseBroadcaster();

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
  ASSERT_EQ(state_interface_conf.names.size(), 6lu);
}
TEST_F(PoseBroadcasterTest, Activate_Success)
{
  SetUpPoseBroadcaster();

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
    ASSERT_EQ(state_interface_conf.names.size(), 6lu);
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
    ASSERT_EQ(state_interface_conf.names.size(), 6lu);  // Should not change when deactivating
  }
}

TEST_F(PoseBroadcasterTest, Update_Success)
{
  SetUpPoseBroadcaster();

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

TEST_F(PoseBroadcasterTest, PublishSuccess)
{
  SetUpPoseBroadcaster();

  // Configure and activate controller
  ASSERT_EQ(
    diagnostic_broadcaster_->on_configure(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);
  ASSERT_EQ(
    diagnostic_broadcaster_->on_activate(rclcpp_lifecycle::State{}),
    controller_interface::CallbackReturn::SUCCESS);

  // Subscribe to diagnostic topic
  diagnostic_msgs::diagnostics diagnostic_msg;
  subscribe_and_get_message("~/diagnostics", diagnostic_msg);

  // Verify content of diagnostic message
  EXPECT_EQ(diagnostic_msg.header.frame_id, "Diagnostics");
  EXPECT_EQ(diagnost);
}

int main(int argc, char * argv[])
{
  ::testing::InitGoogleMock(&argc, argv);
  rclcpp::init(argc, argv);

  int result = RUN_ALL_TESTS();
  rclcpp::shutdown();

  return result;
}