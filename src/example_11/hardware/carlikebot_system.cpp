// Copyright 2021 ros2_control Development Team
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "ros2_control_demo_example_11/carlikebot_system.hpp"

#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <vector>
#include <iomanip>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"


namespace ros2_control_demo_example_11
{
  
hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }
  position_min_ = std::stod(info_.hardware_parameters["steering_position_min"]);
  position_max_ = std::stod(info_.hardware_parameters["steering_position_max"]);
  velocity_min_ = std::stod(info_.hardware_parameters["traction_velocity_min"]);
  velocity_max_ = std::stod(info_.hardware_parameters["traction_velocity_max"]);
  cfg_.device = info_.hardware_parameters["device"];
  cfg_.baud_rate = std::stoi(info_.hardware_parameters["baud_rate"]);
  cfg_.timeout = std::stoi(info_.hardware_parameters["timeout"]);
  // Check if the number of joints is correct based on the mode of operation
  if (info_.joints.size() != 2)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("CarlikeBotSystemHardware"),
      "CarlikeBotSystemHardware::on_init() - Failed to initialize, "
      "because the number of joints %ld is not 2.",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    bool joint_is_steering = joint.name.find("front") != std::string::npos;

    // Steering joints have a position command interface and a position state interface
    if (joint_is_steering)
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a steering joint.",
        joint.name.c_str());

      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 1 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
    else
    {
      RCLCPP_INFO(
        rclcpp::get_logger("CarlikeBotSystemHardware"), "Joint '%s' is a drive joint.",
        joint.name.c_str());

      // Drive joints have a velocity command interface and a velocity state interface
      if (joint.command_interfaces.size() != 1)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
          joint.command_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s command interface. '%s' expected.", joint.name.c_str(),
          joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces.size() != 2)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
          joint.state_interfaces.size());
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
        return hardware_interface::CallbackReturn::ERROR;
      }

      if (joint.state_interfaces[1].name != hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_FATAL(
          rclcpp::get_logger("CarlikeBotSystemHardware"),
          "Joint '%s' has %s state interface. '%s' expected.", joint.name.c_str(),
          joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_POSITION);
        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  // // BEGIN: This part here is for exemplary purposes - Please do not copy to your production
  // code
  hw_start_sec_ = std::stod(info_.hardware_parameters["example_param_hw_start_duration_sec"]);
  hw_stop_sec_ = std::stod(info_.hardware_parameters["example_param_hw_stop_duration_sec"]);
  // // END: This part here is for exemplary purposes - Please do not copy to your production code

  hw_interfaces_["steering"] = Joint("virtual_front_wheel_joint");

  hw_interfaces_["traction"] = Joint("virtual_rear_wheel_joint");

  serial_conn_.setPort(cfg_.device);
  serial_conn_.setBaudrate(cfg_.baud_rate);
  serial::Timeout tt = serial::Timeout::simpleTimeout(cfg_.timeout);
  serial_conn_.setTimeout(tt);
  try
  {
    serial_conn_.open();
  }
  catch (const serial::IOException &e)
  {
    RCLCPP_FATAL(rclcpp::get_logger("CarlikeBotSystemHardware"),
                 "Unable to open port %s: %s",
                 cfg_.device.c_str(), e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> CarlikeBotSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      joint.second.joint_name, hardware_interface::HW_IF_POSITION, &joint.second.state.position));

    if (joint.first == "traction")
    {
      state_interfaces.emplace_back(hardware_interface::StateInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY, &joint.second.state.velocity));
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu state interfaces.",
    state_interfaces.size());

  for (auto s : state_interfaces)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported state interface '%s'.",
      s.get_name().c_str());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CarlikeBotSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (auto & joint : hw_interfaces_)
  {
    if (joint.first == "steering")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_POSITION,
        &joint.second.command.position));
    }
    else if (joint.first == "traction")
    {
      command_interfaces.emplace_back(hardware_interface::CommandInterface(
        joint.second.joint_name, hardware_interface::HW_IF_VELOCITY,
        &joint.second.command.velocity));
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported %zu command interfaces.",
    command_interfaces.size());

  for (auto i = 0u; i < command_interfaces.size(); i++)
  {
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "Exported command interface '%s'.",
      command_interfaces[i].get_name().c_str());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Activating ...please wait...");

  for (auto i = 0; i < hw_start_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_start_sec_ - i);
  }

  for (auto & joint : hw_interfaces_)
  {
    joint.second.state.position = 0.0;

    if (joint.first == "traction")
    {
      joint.second.state.velocity = 0.0;
      joint.second.command.velocity = 0.0;
    }

    else if (joint.first == "steering")
    {
      joint.second.command.position = 0.0;
    }
  }
    
    RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully activated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn CarlikeBotSystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // BEGIN: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Deactivating ...please wait...");

  for (auto i = 0; i < hw_stop_sec_; i++)
  {
    rclcpp::sleep_for(std::chrono::seconds(1));
    RCLCPP_INFO(
      rclcpp::get_logger("CarlikeBotSystemHardware"), "%.1f seconds left...", hw_stop_sec_ - i);
  }
  // END: This part here is for exemplary purposes - Please do not copy to your production code
  RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Successfully deactivated!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type CarlikeBotSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  auto new_time = std::chrono::system_clock::now();
  std::chrono::duration<double> diff = new_time - time_;
  double deltaSeconds = diff.count();
  time_ = new_time;
 
// Read from serial connection
  std::string received_data;
  double pos_prev = hw_interfaces_["traction"].state.position;
  if (serial_conn_.isOpen() && serial_conn_.available())
  {
    try
    {
      received_data = serial_conn_.readline();
      // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Received: %s", received_data.c_str());
      size_t separator = received_data.find(';');
      if (separator != std::string::npos)
      {
        std::string angle_str = received_data.substr(0, separator);
        std::string speed_str = received_data.substr(separator + 1);

        try
        {
          double angle_value = std::stod(angle_str);
          double encoder_position = std::stod(speed_str);
          if (angle_value < position_min_)
          {
            angle_value = position_min_;
          }
          else if (angle_value > position_max_)
          {
            angle_value = position_max_;
          }
          hw_interfaces_["steering"].state.position = angle_value;
          hw_interfaces_["traction"].state.position = encoder_position;
          hw_interfaces_["traction"].state.velocity = (hw_interfaces_["traction"].state.position - pos_prev) / deltaSeconds;
        }
        catch (const std::exception &e)
        {
          RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Failed to parse encoder data: %s", e.what());
          return hardware_interface::return_type::ERROR;
        }
      }
    }
    catch (const serial::IOException &e)
    {
      RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Serial read error: %s", e.what());
      return hardware_interface::return_type::ERROR;
    }
    
  }
  else{
    // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Can't Received Anything");
    hw_interfaces_["steering"].state.position = hw_interfaces_["steering"].command.position;
    hw_interfaces_["traction"].state.velocity = hw_interfaces_["traction"].command.velocity;
    hw_interfaces_["traction"].state.position +=
      hw_interfaces_["traction"].state.velocity * period.seconds();
      
    if (hw_interfaces_["steering"].state.position < position_min_)
    {
      hw_interfaces_["steering"].state.position = position_min_;
    }
    else if (hw_interfaces_["steering"].state.position > position_max_)
    {
      hw_interfaces_["steering"].state.position = position_max_;
    }

    if (hw_interfaces_["traction"].state.velocity < velocity_min_)
    {
      hw_interfaces_["traction"].state.velocity = velocity_min_;
    }
    else if (hw_interfaces_["traction"].state.velocity > velocity_max_)
    {
      hw_interfaces_["traction"].state.velocity = velocity_max_;
    }
  }
  
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ros2_control_demo_example_11 ::CarlikeBotSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double position_command = hw_interfaces_["steering"].command.position;

  if (position_command < position_min_)
  {
    position_command = position_min_;
  }
  else if (position_command > position_max_)
  {
    position_command = position_max_;
  }

  hw_interfaces_["steering"].command.position = position_command;

  double velocity_command = hw_interfaces_["traction"].command.velocity;

  if (velocity_command < velocity_min_)
  {
    velocity_command = velocity_min_;
  }
  else if (velocity_command > velocity_max_)
  {
    velocity_command = velocity_max_;
  }

  hw_interfaces_["traction"].command.velocity = velocity_command;

  // Send command if needed
  if (position_command != previous_position_command_ || velocity_command != previous_velocity_command_)
  {
    std::ostringstream formatted_command;
    formatted_command << std::fixed << std::setprecision(3)
                      << position_command << ";" << velocity_command << "\n";

    if (serial_conn_.isOpen())
    {
      try
      {
        serial_conn_.write(formatted_command.str());
        // RCLCPP_INFO(rclcpp::get_logger("CarlikeBotSystemHardware"), "Sending command: %s", formatted_command.str().c_str());
      }
      catch (const serial::IOException &e)
      {
        RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Serial write error: %s", e.what());
        return hardware_interface::return_type::ERROR;
      }
    }
    else
    {
      RCLCPP_ERROR(rclcpp::get_logger("CarlikeBotSystemHardware"), "Serial port is not open.");
      return hardware_interface::return_type::ERROR;
    }

    // Update previous commands to the current ones
    previous_position_command_ = position_command;
    previous_velocity_command_ = velocity_command;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace ros2_control_demo_example_11

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  ros2_control_demo_example_11::CarlikeBotSystemHardware, hardware_interface::SystemInterface)