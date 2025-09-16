#ifndef TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_
#define TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_

#include "diagnostic_msgs/msg/diagnostics.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"
#include "visibility_control.hpp"

#include "hardware_interface/loaned_state_interface.hpp"
#include "diagnostic_broadcaster/diagnostic_parameters.hpp"

namespace diagnostic_broadcaster
{
    const std::string DEFAULT_DIAGNOSTIC_TOPIC = "~/diagnostics";

    class DiagnosticBroadcaster : public controller_interface::ControllerInterface
    {
    public:
        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        DiagnosticBroadcaster();

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::InterfaceConfiguration
        command_interface_configuration() const override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::InterfaceConfiguration
        state_interface_configuration() const override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_init() override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State &previous_state) override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State &previous_state) override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State &previous_state) override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        controller_interface::return_type update(
            const rclcpp::Time &time, const rclcpp::Duration &period) override;

        DIAGNOSTIC_BROADCASTER__VISIBILITY_PUBLIC
        const std::vector<std::string> &get_joint_names() const { return joint_names_; }

    protected:
        bool has_a_key(const std::string &interface_name);
        bool init_joint_data();
        void init_realtime_publisher_msg();

    protected:
        std::shared_ptr<rclcpp_lifecycle::LifecycleNode> lifecycle_node_;

        const double k_uninitialized_value_ = std::numeric_limits<double>::quiet_NaN();

        std::vector<std::string> joint_names_ = {};
        int joint_num_ = 0;

        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        using loaned_state_interfaces_t = std::vector<std::reference_wrapper<hardware_interface::LoanedStateInterface>>;

        loaned_state_interfaces_t temperature_interfaces_;
        double previous_temp_val_;

        loaned_state_interfaces_t fault_interfaces_;
        double previous_fault_val_;

        rclcpp::Publisher<diagnostic_msgs::msg::Diagnostics>::SharedPtr diagnostic_publisher_;

        std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::Diagnostics>>
            realtime_publisher_;
    };
}
#endif
