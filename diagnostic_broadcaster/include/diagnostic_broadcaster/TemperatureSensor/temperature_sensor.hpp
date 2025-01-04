
#ifndef SEMANTIC_COMPONENTS__TEMPERATURE_SENSOR_HPP_
#define SEMANTIC_COMPONENTS__TEMPERATURE_SENSOR_HPP_

#include <limits>
#include <string>
#include <vector>

#include "semantic_components/semantic_component_interface.hpp"
#include "temperature_msgs/msg/TemperatureBroadcast.hpp"

namespace semantic_components
{
class TemperatureSensor : public SemanticComponentInterface<temperature_msgs::msg::TemperatureBroadcast>
{
public:
  explicit TemperatureSensor(const std::string & name)
  : SemanticComponentInterface(
      name, {{name + "/" + "temperature"}})
  {
  }

  double get_temperature() const
  {
    double temperature;
    temperature = state_interfaces_[0].get().get_value();
    return temperature;
  }

  bool get_values_as_message(string &_name, double &temperature) const
  {
    temperature = get_temperature();
    _name = this->name_; //Protected variable from semantic_components/semantic_component_interface.hpp
    return true;
  }
};

}  

#endif  