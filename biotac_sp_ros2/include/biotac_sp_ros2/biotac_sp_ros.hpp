#ifndef BIOTAC_SP_ROS_HPP_
#define BIOTAC_SP_ROS_HPP_

// Define the maximum number of BioTacs per Cheetah device
#define MAX_BIOTACS_PER_CHEETAH 3 // Or 5, depending on the hardware

// You might want to include ROS message headers here if your
// C++ code directly uses custom message types defined in your package.
// For example:
// #include "biotac_sp_ros/msg/floats.hpp"

// If you have any custom structures or enums related to your BioTac
// interface, you can define them here. For instance:
/*
namespace biotac_sp_ros
{
enum class SensorType
{
  ELECTRODES,
  PRESSURE,
  TEMPERATURE
};

struct BiotacData
{
  rclcpp::Time timestamp;
  std::vector<float> electrode_data;
  float pressure;
  float temperature;
};
} // namespace biotac_sp_ros
*/

#endif // BIOTAC_SP_ROS_HPP_
