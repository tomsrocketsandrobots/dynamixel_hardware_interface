// Copyright 2024 ROBOTIS CO., LTD.
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
//
// Authors: Hye-Jong KIM, Sungho Woo, Woojin Wie

#include "dynamixel_hardware_interface/dynamixel_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <unordered_map>
#include <iomanip>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace dynamixel_hardware_interface
{

DynamixelHardware::DynamixelHardware()
: rclcpp::Node("dynamixel_hardware_interface"),
  logger_(rclcpp::get_logger("dynamixel_hardware_interface"))
{
  dxl_status_ = DXL_OK;
  dxl_torque_status_ = TORQUE_DISABLED;  // start pessimistic; will enable explicitly
  err_timeout_ms_ = 500;
  is_read_in_error_ = false;
  is_write_in_error_ = false;
  read_error_duration_ = rclcpp::Duration(0, 0);
  write_error_duration_ = rclcpp::Duration(0, 0);
}

DynamixelHardware::~DynamixelHardware()
{
  stop();

  if (rclcpp::ok()) {
    RCLCPP_INFO(logger_, "Shutting down ROS2 node...");
  }
}

hardware_interface::CallbackReturn DynamixelHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    RCLCPP_ERROR_STREAM(logger_, "Failed to initialize DynamixelHardware");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.find("number_of_joints") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Required parameter 'number_of_joints' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  num_of_joints_ = static_cast<size_t>(stoi(info_.hardware_parameters["number_of_joints"]));

  try {
    transmission_loader_ = std::make_unique<pluginlib::ClassLoader<transmission_interface::TransmissionLoader>>(
      "transmission_interface", "transmission_interface::TransmissionLoader");
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Failed to create transmission loader: " << e.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.transmissions.empty()) {
    RCLCPP_WARN(logger_, "RobotHWInfo provided no transmissions for this hardware component");
  } else {
    std::ostringstream oss;
    oss << "RobotHWInfo has " << info_.transmissions.size() << " transmission(s): ";
    for (size_t i = 0; i < info_.transmissions.size(); ++i) {
      oss << info_.transmissions[i].name << " (" << info_.transmissions[i].type << ")";
      if (i + 1 < info_.transmissions.size()) {oss << ", ";}
    }
    RCLCPP_INFO_STREAM(logger_, oss.str());
  }

  if (info_.hardware_parameters.find("port_name") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(logger_, "Required parameter 'port_name' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  if (info_.hardware_parameters.find("baud_rate") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(logger_, "Required parameter 'baud_rate' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }

  port_name_ = info_.hardware_parameters["port_name"];
  baud_rate_ = info_.hardware_parameters["baud_rate"];
  if (info_.hardware_parameters.find("error_timeout_ms") != info_.hardware_parameters.end()) {
    try {
      err_timeout_ms_ = stod(info_.hardware_parameters["error_timeout_ms"]);
    } catch (const std::exception & e) {
      RCLCPP_ERROR(
        logger_, "Failed to parse error_timeout_ms parameter: %s, using default value",
        e.what());
    }
  } else {
    RCLCPP_INFO(logger_, "error_timeout_ms parameter not found, using default value of 500ms");
  }

  // Add new parameter for torque initialization
  bool disable_torque_at_init = false;
  if (info_.hardware_parameters.find("disable_torque_at_init") != info_.hardware_parameters.end()) {
    disable_torque_at_init = info_.hardware_parameters.at("disable_torque_at_init") == "true";
    if (disable_torque_at_init) {
      RCLCPP_INFO(
        logger_,
        "Torque will be disabled during initialization if it is enabled at initialization.");
    }
  } else {
    RCLCPP_INFO(
      logger_,
      "If there is a torque enabled Dynamixel, the program will be terminated. Set "
      "'disable_torque_at_init' parameter to 'true' to disable torque at initialization.");
  }

  RCLCPP_INFO_STREAM(
    logger_,
    "port_name " << port_name_.c_str() << " / baudrate " << baud_rate_.c_str());

  if (info_.hardware_parameters.find("dynamixel_model_folder") == info_.hardware_parameters.end()) {
    RCLCPP_ERROR_STREAM(
      logger_,
      "Required parameter 'dynamixel_model_folder' not found in hardware parameters");
    return hardware_interface::CallbackReturn::ERROR;
  }
  std::string dxl_model_folder = info_.hardware_parameters["dynamixel_model_folder"];
  dxl_comm_ = std::unique_ptr<Dynamixel>(
    new Dynamixel(
      (ament_index_cpp::get_package_share_directory("dynamixel_hardware_interface") +
      dxl_model_folder).c_str()));

  RCLCPP_INFO_STREAM(logger_, "$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$");
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Comm Port");

  if (info_.hardware_parameters.find("use_revolute_to_prismatic_gripper") !=
    info_.hardware_parameters.end())
  {
    use_revolute_to_prismatic_ =
      std::stoi(info_.hardware_parameters.at("use_revolute_to_prismatic_gripper")) != 0;
  }

  if (use_revolute_to_prismatic_) {
    RCLCPP_INFO(logger_, "Revolute to Prismatic gripper conversion enabled.");
    initRevoluteToPrismaticParam();
  }
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.find("ID") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "ID is not found in gpio parameters");
      exit(-1);
    }
    if (gpio.parameters.find("type") == gpio.parameters.end()) {
      RCLCPP_ERROR_STREAM(logger_, "type is not found in gpio parameters");
      exit(-1);
    }

    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
    uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
      static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;
    const std::string & type = gpio.parameters.at("type");

    if (type == "dxl") {
      dxl_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "sensor") {
      sensor_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "controller") {
      controller_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "virtual_dxl") {
      dxl_comm_->ReadDxlModelFile(
        comm_id, id,
        static_cast<uint16_t>(stoi(gpio.parameters.at("model_num"))));
      virtual_dxl_comm_id_id_.emplace_back(comm_id, id);
    } else if (type == "virtual_sensor") {
      dxl_comm_->ReadDxlModelFile(
        comm_id, id,
        static_cast<uint16_t>(stoi(gpio.parameters.at("model_num"))));
    } else {
      RCLCPP_ERROR_STREAM(logger_, "Invalid DXL / Sensor type");
      exit(-1);
    }
  }

  if (dxl_comm_->SetupPort(port_name_, baud_rate_) != DxlError::OK) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<uint8_t> reboot_dxl;
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    if (gpio.parameters.find("Reboot") != gpio.parameters.end() &&
      std::stoi(gpio.parameters.at("Reboot")) != 0)
    {
      reboot_dxl.push_back(static_cast<uint8_t>(stoi(gpio.parameters.at("ID"))));
    }
  }
  for (uint8_t id : reboot_dxl) {
    for (int attempt = 0; attempt < 5; ++attempt) {
      if (dxl_comm_->Reboot(id) == DxlError::OK) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(250));
    }
  }

  torque_enabled_comm_id_id_.clear();
  for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
    uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
    uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
      static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;
    const std::string & type = gpio.parameters.at("type");

    bool requires_ping = (type == "controller" || type == "dxl" || type == "sensor");
    if (requires_ping) {
      bool success = false;
      for (int attempt = 0; attempt < 10; ++attempt) {
        if (dxl_comm_->InitDxlComm(comm_id, id) == DxlError::OK) {
          success = true;
          break;
        }
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }
      if (!success) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "[comm_id:" << static_cast<int>(comm_id) <<
            "][ID:" << static_cast<int>(id) <<
            "] Cannot connect communication port! :(");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (type == "dxl" || type == "virtual_dxl") {
      std::vector<std::pair<uint8_t, uint8_t>> single_pair = {{comm_id, id}};
      if (dxl_comm_->InitTorqueStates(single_pair, disable_torque_at_init) != DxlError::OK) {
        RCLCPP_ERROR_STREAM(
          logger_,
          "[comm_id:" << static_cast<int>(comm_id) <<
            "][ID:" << static_cast<int>(id) << "] Error: InitTorqueStates");
        return hardware_interface::CallbackReturn::ERROR;
      }
    }

    if (!InitItem(gpio)) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "[comm_id:" << static_cast<int>(comm_id) <<
          "][ID:" << static_cast<int>(id) << "] Error: InitItem");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (!InitDxlReadItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlReadItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!InitDxlWriteItems()) {
    RCLCPP_ERROR_STREAM(logger_, "Error: InitDxlWriteItems");
    return hardware_interface::CallbackReturn::ERROR;
  }

  dxl_status_ = DXL_OK;

  hdl_joint_states_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_state;
    temp_state.name = joint.name;

    for (auto it : joint.state_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name &&
        HW_IF_HARDWARE_STATE != it.name &&
        HW_IF_TORQUE_ENABLE != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint state interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_states_.push_back(temp_state);
  }

  hdl_joint_commands_.clear();
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    HandlerVarType temp_cmd;
    temp_cmd.name = joint.name;

    for (auto it : joint.command_interfaces) {
      if (hardware_interface::HW_IF_POSITION != it.name &&
        hardware_interface::HW_IF_VELOCITY != it.name &&
        hardware_interface::HW_IF_ACCELERATION != it.name &&
        hardware_interface::HW_IF_EFFORT != it.name)
      {
        RCLCPP_ERROR_STREAM(
          logger_, "Error: invalid joint command interface " << it.name);
        return hardware_interface::CallbackReturn::ERROR;
      }
      temp_cmd.interface_name_vec.push_back(it.name);
      temp_cmd.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_joint_commands_.push_back(temp_cmd);
  }

  if (num_of_joints_ != hdl_joint_commands_.size() &&
    num_of_joints_ != hdl_joint_states_.size())
  {
    RCLCPP_ERROR_STREAM(
      logger_, "Error: number of joints " << num_of_joints_ << ", " <<
        hdl_joint_commands_.size() << ", " << hdl_joint_commands_.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Load transmission plugins now that joint/actuator handles are populated.
  transmissions_loaded_ = load_transmissions_from_urdf();

  if (!transmissions_loaded_) {
    if (info_.hardware_parameters.find("number_of_transmissions") ==
      info_.hardware_parameters.end())
    {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Required parameter 'number_of_transmissions' not found in hardware parameters");
      return hardware_interface::CallbackReturn::ERROR;
    }

    num_of_transmissions_ =
      static_cast<size_t>(stoi(info_.hardware_parameters["number_of_transmissions"]));

    if (!SetMatrix()) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to set transmission matrices");
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    num_of_transmissions_ = info_.transmissions.size();
    RCLCPP_INFO_STREAM(logger_, "Loaded " << num_of_transmissions_ << " transmission(s) from URDF");
  }

  // Validate handle counts against transmission count to catch configuration errors early.
  if (!transmissions_loaded_) {
    if (num_of_transmissions_ != hdl_trans_commands_.size() ||
      num_of_transmissions_ != hdl_trans_states_.size())
    {
      RCLCPP_ERROR_STREAM(
        logger_, "Error: number of transmissions (" << num_of_transmissions_ << ") does not match DXL handle sets (cmd " <<
          hdl_trans_commands_.size() << ", state " << hdl_trans_states_.size() << ")");
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  hdl_sensor_states_.clear();
  for (const hardware_interface::ComponentInfo & sensor : info_.sensors) {
    HandlerVarType temp_state;
    temp_state.name = sensor.name;

    for (auto it : sensor.state_interfaces) {
      temp_state.interface_name_vec.push_back(it.name);
      temp_state.value_ptr_vec.push_back(std::make_shared<double>(0.0));
    }
    hdl_sensor_states_.push_back(temp_state);
  }

  std::string str_dxl_state_pub_name = "dynamixel_hardware_interface/dxl_state";
  if (info_.hardware_parameters.find("dynamixel_state_pub_msg_name") !=
    info_.hardware_parameters.end())
  {
    str_dxl_state_pub_name = info_.hardware_parameters["dynamixel_state_pub_msg_name"];
  }
  dxl_state_pub_ = this->create_publisher<DynamixelStateMsg>(
    str_dxl_state_pub_name, rclcpp::SystemDefaultsQoS());
  dxl_state_pub_uni_ptr_ = std::make_unique<StatePublisher>(dxl_state_pub_);

  size_t num_of_pub_data = hdl_trans_states_.size();
  dxl_state_msg_.id.resize(num_of_pub_data);
  dxl_state_msg_.dxl_hw_state.resize(num_of_pub_data);
  dxl_state_msg_.torque_state.resize(num_of_pub_data);

  size_t num_of_command_entries = 0;
  for (const auto & cmd : hdl_trans_commands_) {
    num_of_command_entries += cmd.interface_name_vec.size();
  }
  command_msg_capacity_ = num_of_command_entries;
  dxl_command_msg_.id.resize(command_msg_capacity_);
  dxl_command_msg_.comm_id.resize(command_msg_capacity_);
  dxl_command_msg_.name.resize(command_msg_capacity_);
  dxl_command_msg_.interface.resize(command_msg_capacity_);
  dxl_command_msg_.value.resize(command_msg_capacity_);

  std::string str_dxl_command_pub_name = "dynamixel_hardware_interface/dxl_command";
  if (info_.hardware_parameters.find("dynamixel_command_pub_msg_name") !=
    info_.hardware_parameters.end())
  {
    str_dxl_command_pub_name = info_.hardware_parameters["dynamixel_command_pub_msg_name"];
  }
  dxl_command_pub_ = this->create_publisher<DynamixelCommandMsg>(
    str_dxl_command_pub_name, rclcpp::SystemDefaultsQoS());
  dxl_command_pub_uni_ptr_ = std::make_unique<CommandPublisher>(dxl_command_pub_);

  using namespace std::placeholders;

  // Get Dynamixel data service
  std::string str_get_dxl_data_srv_name = "dynamixel_hardware_interface/get_dxl_data";
  if (info_.hardware_parameters.find("get_dynamixel_data_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_get_dxl_data_srv_name = info_.hardware_parameters["get_dynamixel_data_srv_name"];
  }
  get_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::GetDataFromDxl>(
    str_get_dxl_data_srv_name,
    std::bind(&DynamixelHardware::get_dxl_data_srv_callback, this, _1, _2));

  // Set Dynamixel data service
  std::string str_set_dxl_data_srv_name = "dynamixel_hardware_interface/set_dxl_data";
  if (info_.hardware_parameters.find("set_dynamixel_data_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_set_dxl_data_srv_name = info_.hardware_parameters["set_dynamixel_data_srv_name"];
  }
  set_dxl_data_srv_ = create_service<dynamixel_interfaces::srv::SetDataToDxl>(
    str_set_dxl_data_srv_name,
    std::bind(&DynamixelHardware::set_dxl_data_srv_callback, this, _1, _2));

  // Reboot Dynamixel service
  std::string str_reboot_dxl_srv_name = "dynamixel_hardware_interface/reboot_dxl";
  if (info_.hardware_parameters.find("reboot_dxl_srv_name") != info_.hardware_parameters.end()) {
    str_reboot_dxl_srv_name = info_.hardware_parameters["reboot_dxl_srv_name"];
  }
  reboot_dxl_srv_ = create_service<dynamixel_interfaces::srv::RebootDxl>(
    str_reboot_dxl_srv_name,
    std::bind(&DynamixelHardware::reboot_dxl_srv_callback, this, _1, _2));

  // Set Dynamixel torque service
  std::string str_set_dxl_torque_srv_name = "dynamixel_hardware_interface/set_dxl_torque";
  if (info_.hardware_parameters.find("set_dxl_torque_srv_name") !=
    info_.hardware_parameters.end())
  {
    str_set_dxl_torque_srv_name = info_.hardware_parameters["set_dxl_torque_srv_name"];
  }
  set_dxl_torque_srv_ = create_service<std_srvs::srv::SetBool>(
    str_set_dxl_torque_srv_name,
    std::bind(&DynamixelHardware::set_dxl_torque_srv_callback, this, _1, _2));

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DynamixelHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  try {
    for (auto it : hdl_trans_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_,
            "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_joint_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_sensor_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for sensor " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_gpio_controller_states_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        state_interfaces.emplace_back(
          hardware_interface::StateInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_state_interfaces: " << e.what());
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
DynamixelHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  try {
    for (auto it : hdl_trans_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_joint_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for joint " << it.name <<
              ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
    for (auto it : hdl_gpio_controller_commands_) {
      for (size_t i = 0; i < it.value_ptr_vec.size(); i++) {
        if (i >= it.interface_name_vec.size()) {
          RCLCPP_ERROR_STREAM(
            logger_, "Interface name vector size mismatch for gpio controller " <<
              it.name << ". Expected size: " << it.value_ptr_vec.size() <<
              ", Actual size: " << it.interface_name_vec.size());
          continue;
        }
        command_interfaces.emplace_back(
          hardware_interface::CommandInterface(
            it.name, it.interface_name_vec.at(i), it.value_ptr_vec.at(i).get()));
      }
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR_STREAM(logger_, "Error in export_command_interfaces: " << e.what());
  }

  return command_interfaces;
}

hardware_interface::CallbackReturn DynamixelHardware::on_activate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return start();
}

hardware_interface::CallbackReturn DynamixelHardware::on_deactivate(
  [[maybe_unused]] const rclcpp_lifecycle::State & previous_state)
{
  return stop();
}

hardware_interface::CallbackReturn DynamixelHardware::start()
{
  rclcpp::Time start_time = this->now();
  rclcpp::Duration error_duration(0, 0);
  while (true) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(0.0));
    if (dxl_comm_err_ == DxlError::OK) {
      break;
    }
    error_duration = this->now() - start_time;
    if (error_duration.seconds() * 1000 >= err_timeout_ms_) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Start Fail (Timeout: " << error_duration.seconds() * 1000 << "ms/" <<
          err_timeout_ms_ << "ms): " << Dynamixel::DxlErrorToString(dxl_comm_err_));
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (transmissions_loaded_) {
    try {
      for (auto & tx : state_transmissions_) {
        if (tx) {tx->actuator_to_joint();}
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Transmission state propagation failed: " << e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    CalcTransmissionToJoint();
  }

  apply_revolute_to_prismatic_state();

  SyncJointCommandWithStates();

  if (transmissions_loaded_) {
    try {
      apply_prismatic_to_revolute_command();
      for (auto & tx : command_transmissions_) {
        if (tx) {tx->joint_to_actuator();}
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Transmission command propagation failed: " << e.what());
      return hardware_interface::CallbackReturn::ERROR;
    }
  } else {
    CalcJointToTransmission();
  }

  dxl_comm_->WriteMultiDxlData();

  if (torque_enabled_comm_id_id_.size() > 0) {
    RCLCPP_INFO_STREAM(logger_, "Enabling torque for Dynamixels");
    for (int i = 0; i < 10; i++) {
      if (dxl_comm_->DynamixelEnable(torque_enabled_comm_id_id_) == DxlError::OK) {
        break;
      }
      RCLCPP_ERROR_STREAM(logger_, "Failed to enable torque for Dynamixels, retry...");
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
  }

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Start!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DynamixelHardware::stop()
{
  if (dxl_comm_) {
    dxl_comm_->DynamixelDisable(dxl_comm_id_id_);
    dxl_comm_->DynamixelDisable(virtual_dxl_comm_id_id_);
  } else {
    RCLCPP_ERROR_STREAM(logger_, "Dynamixel Hardware Stop Fail : dxl_comm_ is nullptr");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO_STREAM(logger_, "Dynamixel Hardware Stop!");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DynamixelHardware::read(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double period_ms = period.seconds() * 1000;

  if (dxl_status_ == REBOOTING) {
    RCLCPP_ERROR_STREAM(logger_, "Dynamixel Read Fail : REBOOTING");
    return hardware_interface::return_type::ERROR;
  } else if (dxl_status_ == DXL_OK || dxl_status_ == COMM_ERROR || dxl_status_ == HW_ERROR) {
    dxl_comm_err_ = CheckError(dxl_comm_->ReadMultiDxlData(period_ms));
    if (dxl_comm_err_ != DxlError::OK && dxl_comm_err_ != DxlError::DXL_HARDWARE_ERROR) {
      if (!is_read_in_error_) {
        is_read_in_error_ = true;
        read_error_duration_ = rclcpp::Duration(0, 0);
      }
      read_error_duration_ = read_error_duration_ + period;

      RCLCPP_ERROR_STREAM(
        logger_,
        "Dynamixel Read Fail (Duration: " << read_error_duration_.seconds() * 1000 << "ms/" <<
          err_timeout_ms_ << "ms)");

      if (read_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
        return hardware_interface::return_type::ERROR;
      }
      return hardware_interface::return_type::OK;
    }
    is_read_in_error_ = false;
    read_error_duration_ = rclcpp::Duration(0, 0);
  }

  if (transmissions_loaded_) {
    try {
      for (auto & tx : state_transmissions_) {
        if (tx) {tx->actuator_to_joint();}
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Transmission state propagation failed: " << e.what());
      return hardware_interface::return_type::ERROR;
    }
  } else {
    CalcTransmissionToJoint();
  }

  apply_revolute_to_prismatic_state();

  for (auto sensor : hdl_gpio_sensor_states_) {
    ReadSensorData(sensor);
  }

  dxl_comm_->ReadItemBuf();

  size_t index = 0;
  if (dxl_state_pub_uni_ptr_) {
    dxl_state_msg_.header.stamp = this->now();
    dxl_state_msg_.comm_state = dxl_comm_err_;
    for (auto it : hdl_trans_states_) {
      dxl_state_msg_.id.at(index) = it.id;
      dxl_state_msg_.dxl_hw_state.at(index) = dxl_hw_err_[it.id];
      auto ts_it = dxl_torque_state_.find({it.comm_id, it.id});
      bool ts = (ts_it != dxl_torque_state_.end()) ? ts_it->second : false;
      dxl_state_msg_.torque_state.at(index) = ts;
      index++;
    }
    dxl_state_pub_uni_ptr_->try_publish(dxl_state_msg_);
  }

  if (rclcpp::ok()) {
    rclcpp::spin_some(this->get_node_base_interface());
  }
  return hardware_interface::return_type::OK;
}
hardware_interface::return_type DynamixelHardware::write(
  [[maybe_unused]] const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (dxl_status_ == DXL_OK || dxl_status_ == HW_ERROR) {
    dxl_comm_->WriteItemBuf();

    ChangeDxlTorqueState();

    for (const auto & joint_cmd : hdl_joint_commands_) {
      for (size_t idx = 0; idx < joint_cmd.interface_name_vec.size(); ++idx) {
        if (joint_cmd.interface_name_vec.at(idx) == hardware_interface::HW_IF_VELOCITY) {
          const double v = *joint_cmd.value_ptr_vec.at(idx);
          if (std::abs(v) > 1e-6) {
            RCLCPP_DEBUG_STREAM_THROTTLE(
              logger_, clock_, 500,
              "Joint cmd " << joint_cmd.name << " / " << joint_cmd.interface_name_vec.at(idx)
                << " = " << v);
          }
        }
      }
    }

    if (transmissions_loaded_) {
      try {
        apply_prismatic_to_revolute_command();
        for (auto & tx : command_transmissions_) {
          if (tx) {tx->joint_to_actuator();}
        }

        // Debug: surface actuator command values after transmission mapping
        // so we can confirm non-zero Goal Velocity is being produced.
        for (const auto & cmd : hdl_trans_commands_) {
          for (size_t idx = 0; idx < cmd.interface_name_vec.size(); ++idx) {
            const double v = *cmd.value_ptr_vec.at(idx);
            if (std::abs(v) > 1e-6) {
              RCLCPP_DEBUG_STREAM_THROTTLE(
                logger_, clock_, 500,
                "Actuator cmd " << cmd.name << " / " << cmd.interface_name_vec.at(idx)
                  << " = " << v);
            }
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_ERROR_STREAM(logger_, "Transmission command propagation failed: " << e.what());
        return hardware_interface::return_type::ERROR;
      }
    } else {
      apply_prismatic_to_revolute_command();
      CalcJointToTransmission();
    }

    if (dxl_command_pub_uni_ptr_ && command_msg_capacity_ > 0U) {
      if (dxl_command_msg_.id.size() != command_msg_capacity_) {
        dxl_command_msg_.id.resize(command_msg_capacity_);
        dxl_command_msg_.comm_id.resize(command_msg_capacity_);
        dxl_command_msg_.name.resize(command_msg_capacity_);
        dxl_command_msg_.interface.resize(command_msg_capacity_);
        dxl_command_msg_.value.resize(command_msg_capacity_);
      }

      size_t slot = 0;
      dxl_command_msg_.header.stamp = this->now();
      for (const auto & cmd : hdl_trans_commands_) {
        for (size_t idx = 0; idx < cmd.interface_name_vec.size(); ++idx) {
          if (slot >= dxl_command_msg_.id.size()) {
            break;
          }
          dxl_command_msg_.id.at(slot) = cmd.id;
          dxl_command_msg_.comm_id.at(slot) = cmd.comm_id;
          dxl_command_msg_.name.at(slot) = cmd.name;
          dxl_command_msg_.interface.at(slot) = cmd.interface_name_vec.at(idx);
          const double unit_value = *cmd.value_ptr_vec.at(idx);
          dxl_command_msg_.value.at(slot) = convert_unit_to_raw_count(
            cmd.comm_id,
            cmd.id,
            cmd.interface_name_vec.at(idx),
            unit_value);
          ++slot;
        }
      }

      if (slot != dxl_command_msg_.id.size()) {
        dxl_command_msg_.id.resize(slot);
        dxl_command_msg_.comm_id.resize(slot);
        dxl_command_msg_.name.resize(slot);
        dxl_command_msg_.interface.resize(slot);
        dxl_command_msg_.value.resize(slot);
      }

      dxl_command_pub_uni_ptr_->try_publish(dxl_command_msg_);
    }

    dxl_comm_->WriteMultiDxlData();

    is_write_in_error_ = false;
    write_error_duration_ = rclcpp::Duration(0, 0);

    return hardware_interface::return_type::OK;
  } else {
    write_error_duration_ = write_error_duration_ + period;

    RCLCPP_ERROR_STREAM(
      logger_,
      "Dynamixel Write Fail (Duration: " << write_error_duration_.seconds() * 1000 << "ms/" <<
        err_timeout_ms_ << "ms)");

    if (write_error_duration_.seconds() * 1000 >= err_timeout_ms_) {
      return hardware_interface::return_type::ERROR;
    }
    return hardware_interface::return_type::OK;
  }
}

DxlError DynamixelHardware::CheckError(DxlError dxl_comm_err)
{
  DxlError error_state = DxlError::OK;
  dxl_status_ = DXL_OK;

  // check comm error
  if (dxl_comm_err != DxlError::OK) {
    RCLCPP_ERROR_STREAM_THROTTLE(
      logger_, clock_, 1000,
      "Communication Fail --> " << Dynamixel::DxlErrorToString(dxl_comm_err));
    dxl_status_ = COMM_ERROR;
    return dxl_comm_err;
  }

  // check hardware error
  const size_t trans_state_count = hdl_trans_states_.size();
  for (size_t i = 0; i < trans_state_count; i++) {
    for (size_t j = 0; j < hdl_trans_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_trans_states_.at(i).interface_name_vec.at(j) == "Hardware Error Status") {
        dxl_hw_err_[hdl_trans_states_.at(i).id] =
          static_cast<uint8_t>(*hdl_trans_states_.at(i).value_ptr_vec.at(j));
        uint8_t hw_error_status = static_cast<uint8_t>(dxl_hw_err_[hdl_trans_states_.at(i).id]);
        std::string error_string = "";

        // Check each bit in the hardware error status
        for (int bit = 0; bit < 8; ++bit) {
          if (hw_error_status & (1 << bit)) {
            const HardwareErrorStatusBitInfo * bit_info = get_hardware_error_status_bit_info(bit);
            if (bit_info) {
              error_string += bit_info->label;
              error_string += " (" + std::string(bit_info->description) + ")/ ";
            } else {
              error_string += "Unknown Error Bit " + std::to_string(bit) + "/ ";
            }
          }
        }

        if (!error_string.empty()) {
          RCLCPP_ERROR_STREAM_THROTTLE(
            logger_, clock_, 1000,
            "Dynamixel Hardware Error States [ ID:" <<
              static_cast<int>(hdl_trans_states_.at(i).id) << "] --> " <<
              "0x" << std::hex << static_cast<int>(hw_error_status) << std::dec <<
              " (" << static_cast<int>(hw_error_status) << "): " << error_string);
          dxl_status_ = HW_ERROR;
          error_state = DxlError::DXL_HARDWARE_ERROR;
        }
      }
      if (hdl_trans_states_.at(i).interface_name_vec.at(j) == "Error Code") {
        dxl_error_code_[hdl_trans_states_.at(i).id] =
          static_cast<uint8_t>(*hdl_trans_states_.at(i).value_ptr_vec.at(j));
        uint8_t error_code = static_cast<uint8_t>(dxl_error_code_[hdl_trans_states_.at(i).id]);

        if (error_code != 0x00) {
          const ErrorCodeInfo * error_info = get_error_code_info(error_code);
          if (error_info) {
            RCLCPP_ERROR_STREAM_THROTTLE(
              logger_, clock_, 1000,
              "Dynamixel Error Code [ ID:" <<
                static_cast<int>(hdl_trans_states_.at(i).id) << "] --> " <<
                "0x" << std::hex << static_cast<int>(error_code) << std::dec <<
                " (" << error_info->label << "): " << error_info->description);
          } else {
            RCLCPP_ERROR_STREAM_THROTTLE(
              logger_, clock_, 1000,
              "Dynamixel Error Code [ ID:" <<
                static_cast<int>(hdl_trans_states_.at(i).id) << "] --> " <<
                "0x" << std::hex << static_cast<int>(error_code) << std::dec <<
                " (Unknown Error Code)");
          }
          dxl_status_ = HW_ERROR;
          error_state = DxlError::DXL_HARDWARE_ERROR;
        }
      }
    }
  }

  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < hdl_joint_states_.at(i).interface_name_vec.size(); j++) {
      if (hdl_joint_states_.at(i).interface_name_vec.at(j) == HW_IF_HARDWARE_STATE) {
        *hdl_joint_states_.at(i).value_ptr_vec.at(j) = error_state;
      }
    }
  }

  return error_state;
}

bool DynamixelHardware::CommReset()
{
  dxl_status_ = REBOOTING;
  stop();
  RCLCPP_INFO_STREAM(logger_, "Communication Reset Start");
  dxl_comm_->RWDataReset();

  auto start_time = this->now();
  while ((this->now() - start_time) < rclcpp::Duration(3, 0)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    RCLCPP_INFO_STREAM(logger_, "Reset Start");
    bool result = true;
    for (auto pr : dxl_comm_id_id_) {
      if (dxl_comm_->Reboot(pr.first) != DxlError::OK) {
        RCLCPP_ERROR_STREAM(logger_, "Cannot reboot dynamixel! :(");
        result = false;
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(200));
    }
    if (!result) {continue;}
    // if (!InitControllerItems()) {continue;}
    // if (!InitDxlItems()) {continue;}
    if (!InitDxlReadItems()) {continue;}
    if (!InitDxlWriteItems()) {continue;}

    RCLCPP_INFO_STREAM(logger_, "RESET Success");
    std::this_thread::sleep_for(std::chrono::seconds(1));
    start();
    dxl_status_ = DXL_OK;
    return true;
  }
  RCLCPP_ERROR_STREAM(logger_, "RESET Failure");
  std::this_thread::sleep_for(std::chrono::seconds(1));
  start();
  return false;
}

bool DynamixelHardware::InitItem(const hardware_interface::ComponentInfo & gpio)
{
  uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
  uint8_t comm_id = (gpio.parameters.find("comm_id") != gpio.parameters.end()) ?
    static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id"))) : id;

  auto unit_it = gpio.parameters.find("[unit info]");
  if (unit_it != gpio.parameters.end()) {
    std::string value = unit_it->second;
    std::vector<std::string> entries;
    {
      std::stringstream ss(value);
      std::string token;
      while (std::getline(ss, token, ';')) {
        size_t first = token.find_first_not_of(" \t\r\n");
        if (first != std::string::npos) {
          size_t last = token.find_last_not_of(" \t\r\n");
          token = token.substr(first, last - first + 1);
        } else {
          token.clear();
        }
        if (!token.empty()) {entries.push_back(token);}
      }
      if (entries.empty()) {entries.push_back(value);}
    }
    for (auto & entry : entries) {
      std::stringstream ls(entry);
      std::string line;
      while (std::getline(ls, line)) {
        size_t first = line.find_first_not_of(" \t\r\n");
        if (first != std::string::npos) {
          size_t last = line.find_last_not_of(" \t\r\n");
          line = line.substr(first, last - first + 1);
        } else {
          line.clear();
        }
        if (line.empty()) {continue;}
        std::vector<std::string> parts;
        std::stringstream ps(line);
        std::string p;
        while (std::getline(ps, p, ',')) {
          size_t p_first = p.find_first_not_of(" \t\r\n");
          if (p_first != std::string::npos) {
            size_t p_last = p.find_last_not_of(" \t\r\n");
            p = p.substr(p_first, p_last - p_first + 1);
          } else {
            p.clear();
          }
          parts.push_back(p);
        }
        if (parts.size() < 4) {continue;}
        std::string data_name = parts[0];
        double unit_multiplier = 1.0;
        try {
          unit_multiplier = std::stod(parts[1]);
        } catch (...) {
          unit_multiplier = 1.0;
        }
        bool is_signed = (parts[3] == std::string("signed"));
        double offset = 0.0;
        if (parts.size() >= 5) {
          try {
            offset = std::stod(parts[4]);
          } catch (...) {
            offset = 0.0;
          }
        }
        dxl_comm_->OverrideUnitInfo(comm_id, id, data_name, unit_multiplier, is_signed, offset);
        RCLCPP_INFO_STREAM(
          logger_,
          "[ID:" << std::to_string(id) << "] override [unit info] for '" << data_name
                 << "' = multiplier:" << unit_multiplier << ", signed:" <<
            (is_signed ? "true" : "false")
                 << ", offset:" << offset);
      }
    }
  }

  bool torque_enabled =
    (gpio.parameters.find("type") != gpio.parameters.end() &&
    (gpio.parameters.at("type") == "dxl" || gpio.parameters.at("type") == "virtual_dxl"));
  if (gpio.parameters.find("Torque Enable") != gpio.parameters.end()) {
    torque_enabled = std::stoi(gpio.parameters.at("Torque Enable")) != 0;
  }
  if (torque_enabled) {
    torque_enabled_comm_id_id_.emplace_back(comm_id, id);
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "Operating Mode") {
      RCLCPP_INFO_STREAM(
        logger_,
        "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
          "] item_name: " << param_name.c_str() << "\tdata: " <<
          param.second);
      if (dxl_comm_->WriteItem(
          comm_id, id, param_name,
          static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
      {
        return false;
      }
      uint32_t mode_readback = 0;
      if (dxl_comm_->ReadItem(comm_id, id, param_name, mode_readback) == DxlError::OK) {
        RCLCPP_INFO_STREAM(
          logger_,
          "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
            "] Operating Mode readback = " << mode_readback);
      }
    }
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "ID" || param_name == "type" ||
      param_name == "Torque Enable" || param_name == "Operating Mode" ||
      param_name == "model_num" || param_name == "comm_id" || param_name == "Reboot")
    {
      continue;
    }
    if (param_name.find("Limit") != std::string::npos) {
      RCLCPP_INFO_STREAM(
        logger_,
        "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
          "] item_name: " << param_name.c_str() << "\tdata: " <<
          param.second);
      if (dxl_comm_->WriteItem(
          comm_id, id, param_name,
          static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
      {
        return false;
      }
    }
  }

  for (const auto & param : gpio.parameters) {
    const std::string & param_name = param.first;
    if (param_name == "ID" || param_name == "type" ||
      param_name == "Torque Enable" || param_name == "Operating Mode" ||
      param_name == "model_num" || param_name == "comm_id" || param_name == "Reboot" ||
      param_name == "[unit info]" ||
      param_name.find("Limit") != std::string::npos)
    {
      continue;
    }
    RCLCPP_INFO_STREAM(
      logger_,
      "[InitItem][comm_id:" << std::to_string(comm_id) << "][ID:" << std::to_string(id) <<
        "] item_name: " << param_name.c_str() << "\tdata: " <<
        param.second);
    if (dxl_comm_->WriteItem(
        comm_id, id, param_name,
        static_cast<uint32_t>(stoi(param.second))) != DxlError::OK)
    {
      return false;
    }
  }
  return true;
}

bool DynamixelHardware::InitDxlReadItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Read Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_states_.clear();
    hdl_gpio_sensor_states_.clear();
    hdl_gpio_controller_states_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      } else if (gpio.parameters.at("type") == "sensor") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_sensor;
        temp_sensor.id = id;
        temp_sensor.comm_id = id;
        temp_sensor.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_sensor.interface_name_vec.push_back(it.name);
          temp_sensor.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_sensor_states_.push_back(temp_sensor);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_states_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_read;
        temp_read.id = id;
        temp_read.comm_id = comm_id;
        temp_read.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_read.interface_name_vec.push_back(it.name);
          temp_read.value_ptr_vec.push_back(std::make_shared<double>(0.0));

          if (it.name == "Hardware Error Status") {
            dxl_hw_err_[id] = 0x00;
          }
        }
        hdl_trans_states_.push_back(temp_read);
      } else if (gpio.parameters.at("type") == "virtual_sensor") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_sensor;
        temp_sensor.id = id;
        temp_sensor.comm_id = comm_id;
        temp_sensor.name = gpio.name;

        for (auto it : gpio.state_interfaces) {
          temp_sensor.interface_name_vec.push_back(it.name);
          temp_sensor.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_sensor_states_.push_back(temp_sensor);
      }
    }
    is_set_hdl_ = true;
  }
  for (auto it : hdl_gpio_controller_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_trans_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  for (auto it : hdl_gpio_sensor_states_) {
    if (dxl_comm_->SetDxlReadItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlRead() != DxlError::OK) {
    return false;
  }
  return true;
}

bool DynamixelHardware::InitDxlWriteItems()
{
  RCLCPP_INFO_STREAM(logger_, "$$$$$ Init Dxl Write Items");
  is_set_hdl_ = false;

  if (!is_set_hdl_) {
    hdl_trans_commands_.clear();
    hdl_gpio_controller_commands_.clear();
    for (const hardware_interface::ComponentInfo & gpio : info_.gpios) {
      if (gpio.parameters.at("type") == "dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          // if (it.name != "Goal Position" &&
          //   it.name != "Goal Velocity" &&
          //   it.name != "Goal Current")
          // {
          //   continue;
          // }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      } else if (gpio.parameters.at("type") == "controller") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        HandlerVarType temp_controller;
        temp_controller.id = id;
        temp_controller.comm_id = id;
        temp_controller.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          temp_controller.interface_name_vec.push_back(it.name);
          temp_controller.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_gpio_controller_commands_.push_back(temp_controller);
      } else if (gpio.parameters.at("type") == "virtual_dxl") {
        uint8_t id = static_cast<uint8_t>(stoi(gpio.parameters.at("ID")));
        uint8_t comm_id = static_cast<uint8_t>(stoi(gpio.parameters.at("comm_id")));
        HandlerVarType temp_write;
        temp_write.id = id;
        temp_write.comm_id = comm_id;
        temp_write.name = gpio.name;

        for (auto it : gpio.command_interfaces) {
          // if (it.name != "Goal Position" &&
          //   it.name != "Goal Velocity" &&
          //   it.name != "Goal Current")
          // {
          //   continue;
          // }
          temp_write.interface_name_vec.push_back(it.name);
          temp_write.value_ptr_vec.push_back(std::make_shared<double>(0.0));
        }
        hdl_trans_commands_.push_back(temp_write);
      }
    }
    is_set_hdl_ = true;
  }

  for (auto it : hdl_trans_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }

  for (auto it : hdl_gpio_controller_commands_) {
    if (dxl_comm_->SetDxlWriteItems(
        it.comm_id, it.id, it.interface_name_vec,
        it.value_ptr_vec) != DxlError::OK)
    {
      return false;
    }
  }
  if (dxl_comm_->SetMultiDxlWrite() != DxlError::OK) {
    return false;
  }

  return true;
}

void DynamixelHardware::ReadSensorData(const HandlerVarType & sensor)
{
  for (auto item : sensor.interface_name_vec) {
    for (size_t i = 0; i < hdl_sensor_states_.size(); i++) {
      for (size_t j = 0; j < hdl_sensor_states_.at(i).interface_name_vec.size(); j++) {
        if (hdl_sensor_states_.at(i).name == sensor.name &&
          hdl_sensor_states_.at(i).interface_name_vec.at(j) == item)
        {
          *hdl_sensor_states_.at(i).value_ptr_vec.at(j) = *sensor.value_ptr_vec.at(j);
        }
      }
    }
  }
}

int32_t DynamixelHardware::convert_unit_to_raw_count(
  uint8_t comm_id,
  uint8_t id,
  const std::string & interface_name,
  double unit_value) const
{
  if (!dxl_comm_) {
    return static_cast<int32_t>(std::llround(unit_value));
  }

  auto dxl_info = dxl_comm_->GetDxlInfo();
  uint16_t address = 0;
  uint8_t size = 0;
  if (!dxl_info.GetDxlControlItem(comm_id, id, interface_name, address, size)) {
    return static_cast<int32_t>(std::llround(unit_value));
  }

  bool is_signed = false;
  (void) dxl_info.GetDxlSignType(comm_id, id, interface_name, is_signed);

  double unit_placeholder = 0.0;
  const bool has_unit_info =
    dxl_info.GetDxlUnitValue(comm_id, id, interface_name, unit_placeholder);

  auto round_double = [](double value) -> int32_t
  {
    return static_cast<int32_t>(std::llround(value));
  };

  if (has_unit_info) {
    switch (size) {
      case 1:
        if (is_signed) {
          return static_cast<int32_t>(
            dxl_info.ConvertUnitToValue<int8_t>(comm_id, id, interface_name, unit_value));
        }
        return static_cast<int32_t>(
          dxl_info.ConvertUnitToValue<uint8_t>(comm_id, id, interface_name, unit_value));
      case 2:
        if (is_signed) {
          return static_cast<int32_t>(
            dxl_info.ConvertUnitToValue<int16_t>(comm_id, id, interface_name, unit_value));
        }
        return static_cast<int32_t>(
          dxl_info.ConvertUnitToValue<uint16_t>(comm_id, id, interface_name, unit_value));
      case 4:
        if (is_signed) {
          return dxl_info.ConvertUnitToValue<int32_t>(comm_id, id, interface_name, unit_value);
        }
        return static_cast<int32_t>(
          dxl_info.ConvertUnitToValue<uint32_t>(comm_id, id, interface_name, unit_value));
      default:
        return round_double(unit_value);
    }
  }

  if (interface_name == "Goal Position") {
    return static_cast<int32_t>(dxl_info.ConvertRadianToValue(comm_id, id, unit_value));
  }

  return round_double(unit_value);
}

bool DynamixelHardware::SetMatrix()
{
  std::string str;
  std::vector<double> d_vec;

  // resize storage (number_of_transmissions x number_of_joint)
  transmission_to_joint_matrix_.assign(
    num_of_joints_, std::vector<double>(num_of_transmissions_, 0.0));

  d_vec.clear();
  if (info_.hardware_parameters.find("transmission_to_joint_matrix") ==
    info_.hardware_parameters.end())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "Parameter 'transmission_to_joint_matrix' not provided. Using identity matrix by default.");
    for (size_t i = 0; i < num_of_joints_; i++) {
      for (size_t j = 0; j < num_of_transmissions_; j++) {
        transmission_to_joint_matrix_[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  } else {
    try {
      std::stringstream ss_tj(info_.hardware_parameters["transmission_to_joint_matrix"]);
      while (std::getline(ss_tj, str, ',')) {
        d_vec.push_back(stod(str));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Failed to parse 'transmission_to_joint_matrix': " << e.what());
      return false;
    }
    const size_t expected_tj = num_of_joints_ * num_of_transmissions_;
    if (d_vec.size() != expected_tj) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Parameter 'transmission_to_joint_matrix' has " << d_vec.size()
                                                        << " elements, expected " << expected_tj);
      return false;
    }
    for (size_t i = 0; i < num_of_joints_; i++) {
      for (size_t j = 0; j < num_of_transmissions_; j++) {
        transmission_to_joint_matrix_[i][j] = d_vec.at(i * num_of_transmissions_ + j);
      }
    }
  }

  fprintf(stderr, "transmission_to_joint_matrix_ \n");
  for (size_t i = 0; i < num_of_joints_; i++) {
    for (size_t j = 0; j < num_of_transmissions_; j++) {
      fprintf(stderr, "[%zu][%zu] %lf, ", i, j, transmission_to_joint_matrix_[i][j]);
    }
    fprintf(stderr, "\n");
  }

  joint_to_transmission_matrix_.assign(
    num_of_transmissions_, std::vector<double>(num_of_joints_, 0.0));

  d_vec.clear();
  if (info_.hardware_parameters.find("joint_to_transmission_matrix") ==
    info_.hardware_parameters.end())
  {
    RCLCPP_WARN_STREAM(
      logger_,
      "Parameter 'joint_to_transmission_matrix' not provided. Using identity matrix by default.");
    for (size_t i = 0; i < num_of_transmissions_; i++) {
      for (size_t j = 0; j < num_of_joints_; j++) {
        joint_to_transmission_matrix_[i][j] = (i == j) ? 1.0 : 0.0;
      }
    }
  } else {
    try {
      std::stringstream ss_jt(info_.hardware_parameters["joint_to_transmission_matrix"]);
      while (std::getline(ss_jt, str, ',')) {
        d_vec.push_back(stod(str));
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Failed to parse 'joint_to_transmission_matrix': " << e.what());
      return false;
    }
    const size_t expected_jt = num_of_transmissions_ * num_of_joints_;
    if (d_vec.size() != expected_jt) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "Parameter 'joint_to_transmission_matrix' has " << d_vec.size()
                                                        << " elements, expected " << expected_jt);
      return false;
    }
    for (size_t i = 0; i < num_of_transmissions_; i++) {
      for (size_t j = 0; j < num_of_joints_; j++) {
        joint_to_transmission_matrix_[i][j] = d_vec.at(i * num_of_joints_ + j);
      }
    }
  }


  fprintf(stderr, "joint_to_transmission_matrix_ \n");
  for (size_t i = 0; i < num_of_transmissions_; i++) {
    for (size_t j = 0; j < num_of_joints_; j++) {
      fprintf(stderr, "[%zu][%zu] %lf, ", i, j, joint_to_transmission_matrix_[i][j]);
    }
    fprintf(stderr, "\n");
  }

  return true;
}

void DynamixelHardware::MapInterfaces(
  size_t outer_size,
  size_t inner_size,
  std::vector<HandlerVarType> & outer_handlers,
  std::vector<HandlerVarType> & inner_handlers,
  const std::vector<std::vector<double>> & matrix,
  const std::unordered_map<std::string, std::vector<std::string>> & iface_map,
  const std::string & conversion_iface,
  const std::string & conversion_name,
  std::function<double(double)> conversion)
{
  for (size_t i = 0; i < outer_size; ++i) {
    for (size_t k = 0; k < outer_handlers.at(i).interface_name_vec.size(); ++k) {
      double value = 0.0;
      const std::string & outer_iface = outer_handlers.at(i).interface_name_vec.at(k);
      auto map_it = iface_map.find(outer_iface);
      if (map_it == iface_map.end()) {
        std::ostringstream oss;
        oss << "No mapping found for '" << outer_handlers.at(i).name
            << "', interface '" << outer_iface
            << "'. Skipping. Available mapping keys: [";
        size_t key_count = 0;
        for (const auto & pair : iface_map) {
          oss << pair.first;
          if (++key_count < iface_map.size()) {oss << ", ";}
        }
        oss << "]";
        RCLCPP_DEBUG_STREAM(logger_, oss.str());
        continue;
      }
      const std::vector<std::string> & mapped_ifaces = map_it->second;
      for (size_t j = 0; j < inner_size; ++j) {
        for (const auto & mapped_iface : mapped_ifaces) {
          auto it = std::find(
            inner_handlers.at(j).interface_name_vec.begin(),
            inner_handlers.at(j).interface_name_vec.end(),
            mapped_iface);
          if (it != inner_handlers.at(j).interface_name_vec.end()) {
            size_t idx = std::distance(inner_handlers.at(j).interface_name_vec.begin(), it);
            value += matrix[i][j] * (*inner_handlers.at(j).value_ptr_vec.at(idx));
            break;
          }
        }
      }
      if (!conversion_iface.empty() && !conversion_name.empty() &&
        outer_iface == conversion_iface &&
        outer_handlers.at(i).name == conversion_name &&
        conversion)
      {
        value = conversion(value);
      }
      *outer_handlers.at(i).value_ptr_vec.at(k) = value;
    }
  }
}

void DynamixelHardware::CalcTransmissionToJoint()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::revoluteToPrismatic, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_joints_,
    num_of_transmissions_,
    hdl_joint_states_,
    hdl_trans_states_,
    transmission_to_joint_matrix_,
    dynamixel_hardware_interface::ros2_to_dxl_state_map,
    hardware_interface::HW_IF_POSITION,
    conversion_joint_name_,
    conv
  );
}

void DynamixelHardware::CalcJointToTransmission()
{
  std::function<double(double)> conv = use_revolute_to_prismatic_ ?
    std::function<double(double)>(
    std::bind(&DynamixelHardware::prismaticToRevolute, this, std::placeholders::_1)) :
    std::function<double(double)>();
  this->MapInterfaces(
    num_of_transmissions_,
    num_of_joints_,
    hdl_trans_commands_,
    hdl_joint_commands_,
    joint_to_transmission_matrix_,
    dynamixel_hardware_interface::dxl_to_ros2_cmd_map,
    "Goal Position",
    conversion_dxl_name_,
    conv
  );
}

void DynamixelHardware::SyncJointCommandWithStates()
{
  for (auto & it_states : hdl_joint_states_) {
    for (auto & it_commands : hdl_joint_commands_) {
      if (it_states.name == it_commands.name) {
        std::string pos_cmd_name = hardware_interface::HW_IF_POSITION;
        // Find index in command interfaces
        auto cmd_it = std::find(
          it_commands.interface_name_vec.begin(),
          it_commands.interface_name_vec.end(),
          pos_cmd_name);
        if (cmd_it == it_commands.interface_name_vec.end()) {
          RCLCPP_WARN_STREAM(
            logger_,
            "No position interface found in command interfaces for joint '" <<
              it_commands.name << "'. Skipping sync!");
          continue;
        }
        size_t cmd_idx = std::distance(it_commands.interface_name_vec.begin(), cmd_it);
        // Find index in state interfaces
        auto state_it = std::find(
          it_states.interface_name_vec.begin(),
          it_states.interface_name_vec.end(),
          pos_cmd_name);
        if (state_it == it_states.interface_name_vec.end()) {
          RCLCPP_WARN_STREAM(
            logger_,
            "No position interface found in state interfaces for joint '" <<
              it_states.name << "'. Skipping sync!");
          continue;
        }
        size_t state_idx = std::distance(it_states.interface_name_vec.begin(), state_it);
        // Sync the value
        *it_commands.value_ptr_vec.at(cmd_idx) = *it_states.value_ptr_vec.at(state_idx);
        RCLCPP_INFO_STREAM(
          logger_, "Sync joint state to command (joint: " << it_states.name << ", " <<
            it_commands.interface_name_vec.at(cmd_idx).c_str() << ", " <<
            *it_commands.value_ptr_vec.at(cmd_idx) << " <- " <<
            it_states.interface_name_vec.at(state_idx).c_str() << ", " <<
            *it_states.value_ptr_vec.at(state_idx));
      }
    }
  }
}

void DynamixelHardware::ChangeDxlTorqueState()
{
  if (torque_enabled_comm_id_id_.size() == 0) {
    return;
  }

  if (dxl_torque_status_ == REQUESTED_TO_ENABLE) {
    RCLCPP_WARN_STREAM(logger_, "Requested to enable torque, Enabling torque for all Dynamixels");
    dxl_comm_->DynamixelEnable(torque_enabled_comm_id_id_);
    SyncJointCommandWithStates();
  } else if (dxl_torque_status_ == REQUESTED_TO_DISABLE) {
    RCLCPP_WARN_STREAM(logger_, "Requested to disable torque, Disabling torque for all Dynamixels");
    dxl_comm_->DynamixelDisable(torque_enabled_comm_id_id_);
    SyncJointCommandWithStates();
  }

  // Aggregate across all devices; if any OFF, report DISABLED
  auto torque_state_map = dxl_comm_->GetDxlTorqueState();
  // Cache per-device torque state for publishers
  dxl_torque_state_.clear();
  bool any_off = false;
  for (const auto & single_torque_state : torque_state_map) {
    dxl_torque_state_[{single_torque_state.first.first, single_torque_state.first.second}] =
      (single_torque_state.second == TORQUE_ON);
    if (single_torque_state.second == TORQUE_OFF) {
      any_off = true;
    }
  }

  dxl_torque_status_ = any_off ? TORQUE_DISABLED : TORQUE_ENABLED;
}

std::shared_ptr<double> DynamixelHardware::find_value_ptr(
  const std::string & name,
  const std::vector<std::string> & candidate_interfaces,
  const std::vector<HandlerVarType> & handlers) const
{
  for (const auto & handler : handlers) {
    if (handler.name != name) {
      continue;
    }
    for (const auto & iface : candidate_interfaces) {
      for (size_t idx = 0; idx < handler.interface_name_vec.size(); ++idx) {
        if (handler.interface_name_vec.at(idx) == iface) {
          return handler.value_ptr_vec.at(idx);
        }
      }
    }
  }
  return nullptr;
}

bool DynamixelHardware::build_transmission_handles(
  const hardware_interface::TransmissionInfo & info,
  bool for_state,
  std::vector<transmission_interface::JointHandle> & joint_handles,
  std::vector<transmission_interface::ActuatorHandle> & actuator_handles) const
{
  joint_handles.clear();
  actuator_handles.clear();

  const auto & joint_source = for_state ? hdl_joint_states_ : hdl_joint_commands_;
  const auto & actuator_source = for_state ? hdl_trans_states_ : hdl_trans_commands_;

  for (const auto & joint : info.joints) {
    const auto & iface_list = for_state ? joint.state_interfaces : joint.command_interfaces;
    const std::vector<std::string> requested = iface_list.empty()
      ? std::vector<std::string>{hardware_interface::HW_IF_POSITION}
      : iface_list;

    for (const auto & iface_name : requested) {
      std::vector<std::string> candidate{iface_name};
      auto ptr = find_value_ptr(joint.name, candidate, joint_source);
      if (!ptr) {
        RCLCPP_ERROR_STREAM(logger_, "Could not find joint interface '" << iface_name << "' for joint '" << joint.name << "'");
        return false;
      }
      joint_handles.emplace_back(joint.name, iface_name, ptr.get());
    }
  }

  for (const auto & actuator : info.actuators) {
    const auto & iface_list = for_state ? actuator.state_interfaces : actuator.command_interfaces;
    const std::vector<std::string> requested = iface_list.empty()
      ? std::vector<std::string>{hardware_interface::HW_IF_POSITION}
      : iface_list;

    for (const auto & ros_iface : requested) {
      std::vector<std::string> candidates{ros_iface};
      if (for_state) {
        auto map_it = dynamixel_hardware_interface::ros2_to_dxl_state_map.find(ros_iface);
        if (map_it != dynamixel_hardware_interface::ros2_to_dxl_state_map.end()) {
          candidates = map_it->second;
        }
      } else {
        auto map_it = dynamixel_hardware_interface::ros2_to_dxl_cmd_map.find(ros_iface);
        if (map_it != dynamixel_hardware_interface::ros2_to_dxl_cmd_map.end()) {
          candidates = map_it->second;
        }
      }

      auto ptr = find_value_ptr(actuator.name, candidates, actuator_source);
      if (!ptr) {
        std::ostringstream oss;
        oss << "Could not find actuator interface for '" << actuator.name << "' (searched: ";
        for (size_t i = 0; i < candidates.size(); ++i) {
          oss << candidates[i];
          if (i + 1 < candidates.size()) {oss << ", ";}
        }
        oss << ")";
        RCLCPP_ERROR_STREAM(logger_, oss.str());
        return false;
      }
      actuator_handles.emplace_back(actuator.name, ros_iface, ptr.get());
    }
  }

  return true;
}

bool DynamixelHardware::load_transmissions_from_urdf()
{
  state_transmissions_.clear();
  command_transmissions_.clear();
  state_actuator_handles_.clear();
  state_joint_handles_.clear();
  command_actuator_handles_.clear();
  command_joint_handles_.clear();

  if (info_.transmissions.empty()) {
    RCLCPP_WARN(logger_, "No transmissions in hardware info; falling back to matrix mapping");
    return false;
  }

  for (const auto & tx_info : info_.transmissions) {
    std::shared_ptr<transmission_interface::TransmissionLoader> loader_inst;
    try {
      loader_inst = transmission_loader_->createSharedInstance(tx_info.type);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to instantiate transmission loader for '" << tx_info.type << "': " << e.what());
      return false;
    }

    RCLCPP_INFO_STREAM(
      logger_, "Loading transmission '" << tx_info.name << "' of type '" << tx_info.type << "'");

    auto tx_state = loader_inst->load(tx_info);
    auto tx_command = loader_inst->load(tx_info);
    if (!tx_state || !tx_command) {
      RCLCPP_ERROR_STREAM(logger_, "Transmission loader for '" << tx_info.name << "' returned null transmission");
      return false;
    }

    std::vector<transmission_interface::JointHandle> joint_state_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_state_handles;
    if (!build_transmission_handles(tx_info, true, joint_state_handles, actuator_state_handles)) {
      return false;
    }

    std::vector<transmission_interface::JointHandle> joint_command_handles;
    std::vector<transmission_interface::ActuatorHandle> actuator_command_handles;
    if (!build_transmission_handles(tx_info, false, joint_command_handles, actuator_command_handles)) {
      return false;
    }

    try {
      tx_state->configure(joint_state_handles, actuator_state_handles);
      tx_command->configure(joint_command_handles, actuator_command_handles);
    } catch (const std::exception & e) {
      RCLCPP_ERROR_STREAM(logger_, "Failed to configure transmission '" << tx_info.name << "': " << e.what());
      return false;
    }

    RCLCPP_INFO_STREAM(
      logger_, "Configured transmission '" << tx_info.name << "' with "
      << joint_command_handles.size() << " joint cmd handles and "
      << actuator_command_handles.size() << " actuator cmd handles");

    state_joint_handles_.push_back(joint_state_handles);
    state_actuator_handles_.push_back(actuator_state_handles);
    command_joint_handles_.push_back(joint_command_handles);
    command_actuator_handles_.push_back(actuator_command_handles);

    state_transmissions_.push_back(tx_state);
    command_transmissions_.push_back(tx_command);
  }

  RCLCPP_INFO_STREAM(logger_, "Successfully loaded " << command_transmissions_.size() << " transmission(s) from URDF");

  return true;
}

void DynamixelHardware::apply_revolute_to_prismatic_state()
{
  if (!use_revolute_to_prismatic_) {
    return;
  }

  for (auto & joint : hdl_joint_states_) {
    if (joint.name != conversion_joint_name_) {
      continue;
    }
    for (size_t idx = 0; idx < joint.interface_name_vec.size(); ++idx) {
      if (joint.interface_name_vec.at(idx) == hardware_interface::HW_IF_POSITION) {
        *joint.value_ptr_vec.at(idx) = revoluteToPrismatic(*joint.value_ptr_vec.at(idx));
        return;
      }
    }
  }
}

void DynamixelHardware::apply_prismatic_to_revolute_command()
{
  if (!use_revolute_to_prismatic_) {
    return;
  }

  for (auto & joint : hdl_joint_commands_) {
    if (joint.name != conversion_joint_name_) {
      continue;
    }
    for (size_t idx = 0; idx < joint.interface_name_vec.size(); ++idx) {
      if (joint.interface_name_vec.at(idx) == hardware_interface::HW_IF_POSITION) {
        *joint.value_ptr_vec.at(idx) = prismaticToRevolute(*joint.value_ptr_vec.at(idx));
        return;
      }
    }
  }
}

void DynamixelHardware::get_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::GetDataFromDxl::Response> response)
{
  uint8_t id = static_cast<uint8_t>(request->id);
  std::string name = request->item_name;

  if (dxl_comm_->InsertReadItemBuf(id, name) != DxlError::OK) {
    RCLCPP_ERROR_STREAM(logger_, "get_dxl_data_srv_callback InsertReadItemBuf");

    response->result = false;
    return;
  }
  double timeout_sec = request->timeout_sec;
  if (timeout_sec == 0.0) {
    timeout_sec = 1.0;
  }
  rclcpp::Time t_start = rclcpp::Clock().now();
  while (dxl_comm_->CheckReadItemBuf(id, name) == false) {
    if ((rclcpp::Clock().now() - t_start).seconds() > timeout_sec) {
      RCLCPP_ERROR_STREAM(
        logger_,
        "get_dxl_data_srv_callback Timeout : " << (rclcpp::Clock().now() - t_start).seconds() );
      response->result = false;
      return;
    }
  }

  response->item_data = dxl_comm_->GetReadItemDataBuf(id, name);
  response->result = true;
}

void DynamixelHardware::set_dxl_data_srv_callback(
  const std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::SetDataToDxl::Response> response)
{
  uint8_t dxl_id = static_cast<uint8_t>(request->id);
  uint32_t dxl_data = static_cast<uint32_t>(request->item_data);
  if (dxl_comm_->InsertWriteItemBuf(dxl_id, request->item_name, dxl_data) == DxlError::OK) {
    response->result = true;
  } else {
    response->result = false;
  }
}

void DynamixelHardware::reboot_dxl_srv_callback(
  [[maybe_unused]] const std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Request> request,
  std::shared_ptr<dynamixel_interfaces::srv::RebootDxl::Response> response)
{
  if (CommReset()) {
    response->result = true;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] SUCCESS");
  } else {
    response->result = false;
    RCLCPP_INFO_STREAM(logger_, "[reboot_dxl_srv_callback] FAIL");
  }
}

void DynamixelHardware::set_dxl_torque_srv_callback(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
  std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
  if (request->data) {
    if (dxl_torque_status_ == TORQUE_ENABLED) {
      // Even if we think torque is enabled, re-check the devices and re-enable if any are off.
      auto torque_state_map = dxl_comm_->GetDxlTorqueState();
      bool any_off = false;
      for (const auto & single_state : torque_state_map) {
        if (single_state.second == TORQUE_OFF) {
          any_off = true;
          break;
        }
      }

      if (!any_off) {
        response->success = true;
        response->message = "Already enabled.";
        RCLCPP_INFO_STREAM(logger_, "Requested to enable torque, but already enabled.");
        return;
      }
    }

    dxl_torque_status_ = REQUESTED_TO_ENABLE;
  } else {
    if (dxl_torque_status_ == TORQUE_DISABLED) {
      response->success = true;
      response->message = "Already disabled.";
      RCLCPP_INFO_STREAM(logger_, "Requested to disable torque, but already disabled.");
      return;
    } else {
      dxl_torque_status_ = REQUESTED_TO_DISABLE;
    }
  }
  response->success = true;
  response->message = "Success to write request.";

  // auto start = std::chrono::steady_clock::now();
  // while (std::chrono::steady_clock::now() - start < std::chrono::seconds(1)) {
  //   if (dxl_torque_status_ == TORQUE_ENABLED) {
  //     if (request->data) {
  //       response->success = true;
  //       response->message = "Success to enable.";
  //     } else {
  //       response->success = false;
  //       response->message = "Fail to enable.";
  //     }
  //     return;
  //   } else if (dxl_torque_status_ == TORQUE_DISABLED) {
  //     if (!request->data) {
  //       response->success = true;
  //       response->message = "Success to disable.";
  //     } else {
  //       response->success = false;
  //       response->message = "Fail to disable.";
  //     }
  //     return;
  //   }
  //   // std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // }
  // response->success = false;
  // response->message = "Fail to write requeset. main thread is not running.";
}

void DynamixelHardware::initRevoluteToPrismaticParam()
{
  if (info_.hardware_parameters.find("revolute_to_prismatic_dxl") !=
    info_.hardware_parameters.end())
  {
    conversion_dxl_name_ = info_.hardware_parameters.at("revolute_to_prismatic_dxl");
  }

  if (info_.hardware_parameters.find("revolute_to_prismatic_joint") !=
    info_.hardware_parameters.end())
  {
    conversion_joint_name_ = info_.hardware_parameters.at("revolute_to_prismatic_joint");
  }

  if (info_.hardware_parameters.find("prismatic_min") != info_.hardware_parameters.end()) {
    prismatic_min_ = std::stod(info_.hardware_parameters.at("prismatic_min"));
  }

  if (info_.hardware_parameters.find("prismatic_max") != info_.hardware_parameters.end()) {
    prismatic_max_ = std::stod(info_.hardware_parameters.at("prismatic_max"));
  }

  if (info_.hardware_parameters.find("revolute_min") != info_.hardware_parameters.end()) {
    revolute_min_ = std::stod(info_.hardware_parameters.at("revolute_min"));
  }

  if (info_.hardware_parameters.find("revolute_max") != info_.hardware_parameters.end()) {
    revolute_max_ = std::stod(info_.hardware_parameters.at("revolute_max"));
  }

  conversion_slope_ = (prismatic_max_ - prismatic_min_) / (revolute_max_ - revolute_min_);
  conversion_intercept_ = prismatic_min_ - conversion_slope_ * revolute_min_;
}

double DynamixelHardware::revoluteToPrismatic(double revolute_value)
{
  return conversion_slope_ * revolute_value + conversion_intercept_;
}

double DynamixelHardware::prismaticToRevolute(double prismatic_value)
{
  return (prismatic_value - conversion_intercept_) / conversion_slope_;
}

std::string DynamixelHardware::getErrorSummary(uint8_t id) const
{
  std::ostringstream summary;
  summary << "Dynamixel ID " << static_cast<int>(id) << " Error Summary:\n";

  // Check hardware error status
  auto hw_err_it = dxl_hw_err_.find(id);
  if (hw_err_it != dxl_hw_err_.end() && hw_err_it->second != 0) {
    uint8_t hw_error_status = static_cast<uint8_t>(hw_err_it->second);
    summary << "  Hardware Error Status: 0x" << std::hex << static_cast<int>(hw_error_status)
            << std::dec << " (" << static_cast<int>(hw_error_status) << ")\n";

    for (int bit = 0; bit < 8; ++bit) {
      if (hw_error_status & (1 << bit)) {
        const HardwareErrorStatusBitInfo * bit_info = get_hardware_error_status_bit_info(bit);
        if (bit_info) {
          summary << "    Bit " << bit << ": " << bit_info->label
                  << " - " << bit_info->description << "\n";
        } else {
          summary << "    Bit " << bit << ": Unknown Error\n";
        }
      }
    }
  }

  // Check error code
  auto error_code_it = dxl_error_code_.find(id);
  if (error_code_it != dxl_error_code_.end() && error_code_it->second != 0x00) {
    uint8_t error_code = static_cast<uint8_t>(error_code_it->second);
    summary << "  Error Code: 0x" << std::hex << static_cast<int>(error_code)
            << std::dec << " (" << static_cast<int>(error_code) << ")\n";

    const ErrorCodeInfo * error_info = get_error_code_info(error_code);
    if (error_info) {
      summary << "    " << error_info->label << " - " << error_info->description << "\n";
    } else {
      summary << "    Unknown Error Code\n";
    }
  }

  // Check if no errors
  if ((hw_err_it == dxl_hw_err_.end() || hw_err_it->second == 0) &&
    (error_code_it == dxl_error_code_.end() || error_code_it->second == 0x00))
  {
    summary << "  No errors detected\n";
  }

  return summary.str();
}

std::string DynamixelHardware::getAllErrorSummaries() const
{
  std::ostringstream all_summaries;
  all_summaries << "=== All Dynamixel Error Summaries ===\n";

  // Get all unique IDs from both error maps
  std::set<uint8_t> all_ids;
  for (const auto & pair : dxl_hw_err_) {
    all_ids.insert(pair.first);
  }
  for (const auto & pair : dxl_error_code_) {
    all_ids.insert(pair.first);
  }

  if (all_ids.empty()) {
    all_summaries << "No Dynamixel IDs found in error maps.\n";
  } else {
    for (uint8_t id : all_ids) {
      all_summaries << getErrorSummary(id) << "\n";
    }
  }

  all_summaries << "=====================================\n";
  return all_summaries.str();
}

}  // namespace dynamixel_hardware_interface

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  dynamixel_hardware_interface::DynamixelHardware,
  hardware_interface::SystemInterface
)
