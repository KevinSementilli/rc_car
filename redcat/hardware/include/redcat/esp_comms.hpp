#ifndef REDCAT_ESP_COMMS_HPP
#define REDCAT_ESP_COMMS_HPP

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#include <string>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class ESPComms
{
public:

    ESPComms(const rclcpp::Logger &logger, const std::string &interface_name = "")
        : logger_(logger), interface_name_(interface_name)
    {

    }

    ~ESPComms() { if (fd_ >= 0) { close(fd_); } }

    bool configure(const std::string &interface_name, int bitrate)
    {
        interface_name_ = interface_name;
        bitrate_ = bitrate;

        if (interface_name_.empty()) {
            RCLCPP_ERROR(logger_, "Interface name is empty");
            return false;
        }

        if (bitrate_ <= 0) {
            RCLCPP_ERROR(logger_, "Invalid baud rate: %d", bitrate_);
            return false;
        }

        return true;
    }

    bool connect(const std::string &interface, int bitrate) 
    {

        if (!configure(interface, bitrate)) {
            return false;
        }

        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }

        fd_ = open(interface_name_.c_str(), O_RDWR | O_NOCTTY | O_SYNC);
        if (fd_ < 0) {
            RCLCPP_ERROR(logger_, "Failed to open serial device '%s'", interface_name_.c_str());
            return false;
        }

        struct termios tty {};
        if (tcgetattr(fd_, &tty) != 0) {
            RCLCPP_ERROR(logger_, "Failed to read termios config for '%s'", interface_name_.c_str());
            close(fd_);
            fd_ = -1;
            return false;
        }

        cfmakeraw(&tty);
        tty.c_cflag |= (CLOCAL | CREAD);
        tty.c_cflag &= ~CSIZE;
        tty.c_cflag |= CS8;
        tty.c_cflag &= ~(PARENB | PARODD);
        tty.c_cflag &= ~CSTOPB;
        tty.c_cflag &= ~CRTSCTS;
        tty.c_cc[VMIN] = 0;
        tty.c_cc[VTIME] = 0;

        const speed_t speed = baud_to_termios(bitrate_);
        if (speed == static_cast<speed_t>(0)) {
            RCLCPP_ERROR(logger_, "Unsupported baud rate: %d", bitrate_);
            close(fd_);
            fd_ = -1;
            return false;
        }

        if (cfsetispeed(&tty, speed) != 0 || cfsetospeed(&tty, speed) != 0) {
            RCLCPP_ERROR(logger_, "Failed to set baud rate to %d", bitrate_);
            close(fd_);
            fd_ = -1;
            return false;
        }

        if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
            RCLCPP_ERROR(logger_, "Failed to apply termios config for '%s'", interface_name_.c_str());
            close(fd_);
            fd_ = -1;
            return false;
        }

        tcflush(fd_, TCIOFLUSH);

        return true;
    }

    bool disconnect(const std::string &interface = "") 
    {
        (void)interface;

        if (fd_ >= 0) {
            close(fd_);
            fd_ = -1;
        }

        return true;
    }

    bool send_frame(uint8_t frame_type, const std::vector<uint8_t> &data)
    {

        if (fd_ < 0) {
            RCLCPP_ERROR(logger_, "Cannot send frame before connecting");
            return false;
        }

        if (data.size() > 255) {
            RCLCPP_ERROR(logger_, "Frame payload too large: %zu", data.size());
            return false;
        }

        std::vector<uint8_t> frame;
        frame.reserve(data.size() + 5);
        frame.push_back(kSync0);
        frame.push_back(kSync1);
        frame.push_back(frame_type);
        frame.push_back(static_cast<uint8_t>(data.size()));
        frame.insert(frame.end(), data.begin(), data.end());
        frame.push_back(compute_checksum(frame_type, static_cast<uint8_t>(data.size()), data));

        size_t total_written = 0;
        while (total_written < frame.size()) {
            const ssize_t bytes_written = write(fd_, frame.data() + total_written, frame.size() - total_written);
            if (bytes_written <= 0) {
                return false;
            }
            total_written += static_cast<size_t>(bytes_written);
        }

        return true;

    }

    bool receive_frame(uint8_t &frame_type, std::vector<uint8_t> &payload, int timeout_ms = 100)
    {
        if (fd_ < 0) {
            RCLCPP_ERROR(logger_, "Cannot receive frame before connecting");
            return false;
        }

        int remaining_ms = timeout_ms;
        uint8_t b0 = 0;
        uint8_t b1 = 0;

        while (remaining_ms > 0) {
            if (!read_byte_with_timeout(b0, remaining_ms)) {
                return false;
            }

            if (b0 != kSync0) {
                continue;
            }

            if (!read_byte_with_timeout(b1, remaining_ms)) {
                return false;
            }

            if (b1 != kSync1) {
                if (b1 == kSync0) {
                    b0 = kSync0;
                }
                continue;
            }

            uint8_t length = 0;
            if (!read_byte_with_timeout(frame_type, remaining_ms) ||
                !read_byte_with_timeout(length, remaining_ms))
            {
                return false;
            }

            payload.resize(length);
            for (size_t i = 0; i < payload.size(); ++i) {
                if (!read_byte_with_timeout(payload[i], remaining_ms)) {
                    return false;
                }
            }

            uint8_t checksum = 0;
            if (!read_byte_with_timeout(checksum, remaining_ms)) {
                return false;
            }

            const uint8_t expected = compute_checksum(frame_type, length, payload);
            if (checksum != expected) {
                RCLCPP_WARN(logger_, "Checksum mismatch in received serial frame");
                payload.clear();
                frame_type = 0;
                continue;
            }

            return true;
        }

        return false;
    }

private:

    static constexpr uint8_t kSync0 = 0xAA;
    static constexpr uint8_t kSync1 = 0x55;

    speed_t baud_to_termios(int baud) const
    {
        switch (baud) {
            case 9600: return B9600;
            case 19200: return B19200;
            case 38400: return B38400;
            case 57600: return B57600;
            case 115200: return B115200;
            case 230400: return B230400;
            case 460800: return B460800;
            case 921600: return B921600;
            default: return static_cast<speed_t>(0);
        }
    }

    uint8_t compute_checksum(uint8_t frame_type, uint8_t length, const std::vector<uint8_t> &payload) const
    {
        uint8_t checksum = frame_type ^ length;
        for (const uint8_t byte : payload) {
            checksum ^= byte;
        }
        return checksum;
    }

    bool read_byte_with_timeout(uint8_t &byte, int &remaining_ms)
    {
        if (remaining_ms <= 0) {
            return false;
        }

        fd_set read_fds;
        FD_ZERO(&read_fds);
        FD_SET(fd_, &read_fds);

        struct timeval timeout {};
        timeout.tv_sec = remaining_ms / 1000;
        timeout.tv_usec = (remaining_ms % 1000) * 1000;

        const auto start = std::chrono::steady_clock::now();
        const int ready = select(fd_ + 1, &read_fds, nullptr, nullptr, &timeout);
        const auto end = std::chrono::steady_clock::now();
        const int elapsed = static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count());
        remaining_ms = std::max(0, remaining_ms - elapsed);

        if (ready <= 0) {
            return false;
        }

        const ssize_t bytes_read = read(fd_, &byte, 1);
        return bytes_read == 1;
    }

    rclcpp::Logger logger_;
    std::string interface_name_;
    int bitrate_{0};
    int fd_{-1};

};

#endif // REDCAT_ESP_COMMS_HPP