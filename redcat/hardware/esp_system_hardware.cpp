#include "redcat/esp_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace redcat {

    namespace {
        constexpr uint8_t kHandshakeMsgType = 0x10;
        constexpr uint8_t kCommandMsgType = 0x11;
        constexpr uint8_t kFeedbackMsgType = 0x12;
        constexpr double kScale = 1000.0;

        inline int16_t encode_scaled(double value)
        {
            return static_cast<int16_t>(std::lround(value * kScale));
        }

        inline double decode_scaled(int16_t value)
        {
            return static_cast<double>(value) / kScale;
        }

        inline std::string hardware_param_or_fallback(
            const hardware_interface::HardwareInfo &info,
            const std::string &primary,
            const std::string &fallback)
        {
            const auto primary_it = info.hardware_parameters.find(primary);
            if (primary_it != info.hardware_parameters.end()) {
                return primary_it->second;
            }

            const auto fallback_it = info.hardware_parameters.find(fallback);
            if (fallback_it != info.hardware_parameters.end()) {
                return fallback_it->second;
            }

            return {};
        }
    }

    hardware_interface::CallbackReturn ESPSystemHardware::on_init(
        const hardware_interface::HardwareComponentInterfaceParams & params) {

        if (hardware_interface::HardwareComponentInterface::on_init(params) !=
            hardware_interface::CallbackReturn::SUCCESS) 
        {
            RCLCPP_ERROR(logger_, "info Failed!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        const auto & info = params.hardware_info;

        if (info.joints.size() < 2) {
            RCLCPP_FATAL(logger_, "Expected two joints: driver and steering");
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Extract global hardware parameters
        cfg_.interface_name = hardware_param_or_fallback(info, "interface_name", "device");
        if (cfg_.interface_name.empty()) {
            RCLCPP_FATAL(logger_, "Missing required hardware parameter 'interface_name' or 'device'");
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.baud_rate = std::stoi(info.hardware_parameters.at("baud_rate"));
        cfg_.timeout_ms = std::stoi(info.hardware_parameters.at("timeout_ms"));
        cfg_.loop_rate = std::stoi(info.hardware_parameters.at("loop_rate"));

        cfg_.driver_joint_name = info.joints[0].name;
        cfg_.steering_joint_name = info.joints[1].name;

        RCLCPP_INFO(logger_, "info passed successfully!");

        // Validate command and state interfaces for driver joint
        if (info.joints[0].command_interfaces.size() != 1 ||
            info.joints[0].command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
        {
            RCLCPP_FATAL(
                logger_, "Joint '%s' must have exactly 1 command interface: 'velocity'.",
                info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints[0].state_interfaces.size() != 2 ||
            info.joints[0].state_interfaces[0].name != hardware_interface::HW_IF_POSITION ||
            info.joints[0].state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY) 
        {
            RCLCPP_FATAL(
                logger_, "Joint '%s' must have exactly 2 state interfaces: 'position' and 'velocity'.",
                info.joints[0].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        // Validate command and state interfaces for steering joint
        if (info.joints[1].command_interfaces.size() != 1 ||
            info.joints[1].command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                logger_, "Joint '%s' must have exactly 1 command interface: 'position'.",
                info.joints[1].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (info.joints[1].state_interfaces.size() != 1 ||
            info.joints[1].state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
        {
            RCLCPP_FATAL(
                logger_, "Joint '%s' must have exactly 1 state interface: 'position'.",
                info.joints[1].name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        RCLCPP_INFO(logger_, "All joints have valid command and state interfaces");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ESPSystemHardware::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        RCLCPP_INFO(logger_, "Configuring serial connection...");

        if (!comms_.configure(cfg_.interface_name, cfg_.baud_rate)) {
            RCLCPP_ERROR(logger_, "Failed to configure ESP comms for interface '%s'", cfg_.interface_name.c_str());
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Successfully configured serial connection...");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ESPSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        
        RCLCPP_INFO(logger_, "Establishing serial connection with ESP32: %s, bitrate: %d", cfg_.interface_name.c_str(), cfg_.baud_rate);

        while(!comms_.connect(cfg_.interface_name, cfg_.baud_rate)) {

            RCLCPP_WARN(logger_, "Could not connection to ESP32, retrying ...");
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        std::vector<uint8_t> handshake_frame {0xAC, 0x01};
        if (!comms_.send_frame(kHandshakeMsgType, handshake_frame)) {
            RCLCPP_ERROR(logger_, "Failed to send handshake frame");
            return hardware_interface::CallbackReturn::ERROR;
        }

        uint8_t handshake_type = 0;
        std::vector<uint8_t> handshake_payload;
        if (!comms_.receive_frame(handshake_type, handshake_payload, cfg_.timeout_ms)) {
            RCLCPP_ERROR(logger_, "No handshake ACK received from ESP32");
            return hardware_interface::CallbackReturn::ERROR;
        }

        if (handshake_type != kHandshakeMsgType || handshake_payload.size() < 2 ||
            handshake_payload[0] != 0xAC || handshake_payload[1] == 0x00)
        {
            RCLCPP_ERROR(logger_, "Invalid handshake ACK payload from ESP32");
            return hardware_interface::CallbackReturn::ERROR;
        }

        RCLCPP_INFO(logger_, "Successfully activated!");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn ESPSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {

      RCLCPP_INFO(logger_, "Deactivating hardware interface...");
      comms_.disconnect(cfg_.interface_name);
      RCLCPP_INFO(logger_, "Successfully deactivated!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> ESPSystemHardware::export_state_interfaces() 
    {        
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            cfg_.driver_joint_name, hardware_interface::HW_IF_POSITION, &driver_joint_pos_));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            cfg_.driver_joint_name, hardware_interface::HW_IF_VELOCITY, &driver_joint_vel_));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            cfg_.steering_joint_name, hardware_interface::HW_IF_POSITION, &steering_joint_pos_));

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> ESPSystemHardware::export_command_interfaces() 
    {        
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
                cfg_.driver_joint_name, hardware_interface::HW_IF_VELOCITY, &driver_cmd_));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            cfg_.steering_joint_name, hardware_interface::HW_IF_POSITION, &steering_cmd_));

        return command_interfaces;
    }

    hardware_interface::return_type ESPSystemHardware::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {   
        uint8_t message_type = 0;
        std::vector<uint8_t> feedback_payload;
        if (!comms_.receive_frame(message_type, feedback_payload, cfg_.timeout_ms)) {
            RCLCPP_WARN(logger_, "No feedback received from ESP32");
            return hardware_interface::return_type::ERROR;
        }

        if (message_type != kFeedbackMsgType || feedback_payload.size() < 7) {
            RCLCPP_WARN(logger_, "Unexpected feedback frame received");
            return hardware_interface::return_type::ERROR;
        }

        const bool ack = feedback_payload[0] != 0;
        if (!ack) {
            RCLCPP_WARN(logger_, "ESP32 reported command execution failure");
            return hardware_interface::return_type::ERROR;
        }

        const auto driver_pos_raw = static_cast<int16_t>((static_cast<uint16_t>(feedback_payload[1]) << 8) |
                                                         static_cast<uint16_t>(feedback_payload[2]));
        const auto driver_vel_raw = static_cast<int16_t>((static_cast<uint16_t>(feedback_payload[3]) << 8) |
                                                         static_cast<uint16_t>(feedback_payload[4]));
        const auto steering_pos_raw = static_cast<int16_t>((static_cast<uint16_t>(feedback_payload[5]) << 8) |
                                                           static_cast<uint16_t>(feedback_payload[6]));

        driver_joint_pos_ = decode_scaled(driver_pos_raw);
        driver_joint_vel_ = decode_scaled(driver_vel_raw);
        steering_joint_pos_ = decode_scaled(steering_pos_raw);

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type ESPSystemHardware::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {
        const int16_t driver_cmd_raw = encode_scaled(driver_cmd_);
        const int16_t steering_cmd_raw = encode_scaled(steering_cmd_);

        std::vector<uint8_t> payload {
            0x01,
            static_cast<uint8_t>((driver_cmd_raw >> 8) & 0xFF),
            static_cast<uint8_t>(driver_cmd_raw & 0xFF),
            static_cast<uint8_t>((steering_cmd_raw >> 8) & 0xFF),
            static_cast<uint8_t>(steering_cmd_raw & 0xFF)
        };

        if (!comms_.send_frame(kCommandMsgType, payload)) {
            RCLCPP_ERROR(logger_, "Failed to send command frame to ESP32");
            return hardware_interface::return_type::ERROR;
        }

        return hardware_interface::return_type::OK;
    }

} // namespace redcat

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  redcat::ESPSystemHardware, hardware_interface::SystemInterface)