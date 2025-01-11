#include <gmock/gmock.h>
#include <memory>

#include "controller_manager/controller_manager.hpp"
#include "hardware_interface/resource_manager.hpp"
#include "rclcpp/executor.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/utilities.hpp"
#include "ros2_control_test_assets/descriptions.hpp"

TEST(TestLoadDiagnosticBroadcaster, load_controller)
{
  // Initialize the ROS 2 system
  rclcpp::init(0, nullptr);

  // Create a single-threaded executor
  std::shared_ptr<rclcpp::Executor> executor =
    std::make_shared<rclcpp::executors::SingleThreadedExecutor>();

  // Create a ResourceManager (wymagany przez ControllerManager)
  auto resource_manager = std::make_unique<hardware_interface::ResourceManager>();

  // Create a Controller Manager with a minimal robot URDF
  controller_manager::ControllerManager cm(
    std::move(resource_manager), executor, "test_controller_manager");

  // Attempt to load the DiagnosticBroadcaster controller
  auto controller = cm.load_controller(
    "test_diagnostic_broadcaster", "diagnostic_broadcaster/DiagnosticBroadcaster");

  // Verify the controller was loaded successfully
  ASSERT_NE(controller, nullptr);


  // Cleanup and shutdown ROS 2
  rclcpp::shutdown();
}
