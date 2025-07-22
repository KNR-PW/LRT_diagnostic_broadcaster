#ifndef TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_
#define TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_

#include "diagnostic_msgs/msg/diagnostics.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"

#include "hardware_interface/loaned_state_interface.hpp"

namespace diagnostic_broadcaster
{
    class DiagnosticBroadcaster : public controller_interface::ControllerInterface
    {
    public:
        DiagnosticBroadcaster();

        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;

        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        controller_interface::CallbackReturn on_init() override;

        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        const std::vector<std::string> &get_joint_names() const { return joint_names_; }

        void assign_joints(std::vector<std::string> assigned_state_interfaces);

    protected:
        bool has_any_key(std::string _interface_name);
        bool init_joint_data();
        void init_realtime_publisher_msg();

    protected:
    
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node_;
        
        std::vector<std::string> joint_names_ = {};
        std::vector<std::string> interface_names = {"temperature", "fault"};

        using loaned_state_interfaces_t = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;
        loaned_state_interfaces_t joint_state_interfaces_;

        rclcpp::Publisher<diagnostic_msgs::msg::Diagnostics>::SharedPtr diagnostic_publisher_;

        std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::Diagnostics>>
            realtime_publisher_;
    };
}
#endif
