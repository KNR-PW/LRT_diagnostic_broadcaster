#ifndef TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_
#define TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_

#include <unordered_map>

#include "diagnostic_msgs/msg/diagnostics.hpp"

#include "realtime_tools/realtime_publisher.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"


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

    protected:
        bool has_any_key(std::string _interface_name);
        bool init_joint_data();
        void init_realtime_publisher_msg();
    protected:
        std::vector<std::string> interface_names = {"temperature"}; // DECLARATION OF AN ARRAY WHERE WE POINT WHICH STATE INTERFACE DO WE NEED TO BROADCAST 
        std::vector<std::string> joint_names_;                      // TO ADD MORE INTERFACES YOU NEED TO UPDATE Diagnostic.msg
                                                                    // FOR EXAMPLE: interace_names = {"temperature", "pressure"}
                                                                    // In Diagnostics.msg
                                                                    // add new line:  float64[] pressure;
                                                                    // In diagnostic_broadcaster.cpp 
                                                                    // Add new line in joint_state_init for new message line
                                                                    // and
                                                                    // Only the joints which have minimum one of the interfaces will be broadcasted
        std::unordered_map<std::string, std::unordered_map<std::string, double>> joints_interfaces_values; 

        rclcpp::Publisher<diagnostic_msgs::msg::Diagnostics>::SharedPtr diagnostic_publisher_; 

        std::unique_ptr<realtime_tools::RealtimePublisher<diagnostic_msgs::msg::Diagnostics>> 
        realtime_publisher_;
};
}
#endif

