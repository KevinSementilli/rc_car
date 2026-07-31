#ifndef REDCAT_ESP_SYSTEM_HARDWARE_HPP_
#define REDCAT_ESP_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "redcat/visibility_control.h"
#include "redcat/esp_comms.hpp"

namespace redcat
{   
    class ESPSystemHardware : public hardware_interface::SystemInterface
    {

        struct Config {

            std::string interface_name = "";
            int baud_rate = 0;
            int timeout_ms = 0;
            int loop_rate = 0;

            std::string driver_joint_name = "";
            std::string steering_joint_name = "";
        };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(ESPSystemHardware)

        REDCAT_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareComponentInterfaceParams & params) override;

        REDCAT_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state) override;

        REDCAT_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        REDCAT_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        REDCAT_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        REDCAT_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        REDCAT_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        REDCAT_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:

        Config cfg_;
        rclcpp::Logger logger_ = rclcpp::get_logger("esp_system_hardware");  
        ESPComms comms_{logger_}; 

        double steering_joint_pos_{0.0};
        double driver_joint_pos_{0.0};
        double driver_joint_vel_{0.0};

        double steering_cmd_{0.0};
        double driver_cmd_{0.0};

    };
}

#endif  // REDCAT_ESP_SYSTEM_HARDWARE_HPP_