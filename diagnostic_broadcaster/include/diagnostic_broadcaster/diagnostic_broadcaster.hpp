#ifndef TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_
#define TEMPERATURE_BROADCASTER__TEMPERATURE_BROADCASTER_HPP_


#include "diagnostic_msgs/msg/diagnostic_broadcaster.hpp"
#include "realtime_tools/realtime_publisher.hpp"
#include "controller_interface/controller_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "rclcpp/publisher.hpp"



class TemperatureBroadcaster : public controller_interface::ControllerInterface
{
    public:
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

    private:
        std::shared_ptr<ParamListener> param_listener_;
        Params params_;

        std::unique_ptr<semantic_components::PoseSensor> temperature_sensor_; 

        rclcpp::Publisher<temperature_msgs::msg::TemperatureBroadcast>::SharedPtr temperature_publisher_; 

        std::unique_ptr<realtime_tools::RealtimePublisher<temperature_msgs::msg::TemperatureBroadcast>> 
        realtime_publisher_;

};

#endif

