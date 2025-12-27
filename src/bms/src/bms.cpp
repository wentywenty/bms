#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <vector>
#include <cmath>
#include <cstring>
#include <iostream>

// TWS BMS Protocol Constants
#define BMS_ADDR 0x01
#define FUNC_READ 0x03

class TwsBmsNode : public rclcpp::Node {
public:
    TwsBmsNode() : Node("bms_node"), serial_fd_(-1) {
        // Parameters
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("poll_rate", 1.0);
        this->declare_parameter("frame_id", "battery_link");

        port_name_ = this->get_parameter("port_name").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        double rate = this->get_parameter("poll_rate").as_double();
        frame_id_ = this->get_parameter("frame_id").as_string();

        // Init Serial
        if (open_serial()) {
            RCLCPP_INFO(this->get_logger(), "Serial port %s opened successfully.", port_name_.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port_name_.c_str());
            // Don't exit, allow retry logic or keep node alive for diagnostics
        }

        // Publisher & Timer
        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&TwsBmsNode::timer_callback, this));
    }

    ~TwsBmsNode() {
        if (serial_fd_ >= 0) close(serial_fd_);
    }

private:
    int serial_fd_;
    std::string port_name_;
    int baud_rate_;
    std::string frame_id_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::TimerBase::SharedPtr timer_;

    // Open and configure serial port (8N1, 115200 default)
    bool open_serial() {
        serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
        if (serial_fd_ < 0) return false;

        struct termios options;
        tcgetattr(serial_fd_, &options);

        // Baud rate
        speed_t baud;
        switch(baud_rate_) {
            case 9600: baud = B9600; break;
            case 115200: baud = B115200; break;
            default: baud = B115200; 
        }
        cfsetispeed(&options, baud);
        cfsetospeed(&options, baud);

        // 8N1 [cite: 29-31]
        options.c_cflag &= ~PARENB; // No parity
        options.c_cflag &= ~CSTOPB; // 1 stop bit
        options.c_cflag &= ~CSIZE;
        options.c_cflag |= CS8;     // 8 data bits

        options.c_cflag |= (CLOCAL | CREAD);
        options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // Raw input
        options.c_oflag &= ~OPOST; // Raw output

        tcsetattr(serial_fd_, TCSANOW, &options);
        fcntl(serial_fd_, F_SETFL, 0); // Blocking read (with timeout via select later if needed)
        return true;
    }

    // CRC16 Calculation (Modbus) [cite: 215]
    uint16_t calculate_crc(const uint8_t *data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; i++) {
            crc ^= data[i];
            for (int j = 0; j < 8; j++) {
                if (crc & 1) crc = (crc >> 1) ^ 0xA001;
                else crc >>= 1;
            }
        }
        return crc;
    }

    // Send Modbus Request
    void send_request(uint16_t start_addr, uint16_t num_regs) {
        uint8_t frame[8];
        frame[0] = BMS_ADDR;
        frame[1] = FUNC_READ;
        frame[2] = (start_addr >> 8) & 0xFF; // High byte first for address [cite: 48]
        frame[3] = start_addr & 0xFF;
        frame[4] = (num_regs >> 8) & 0xFF;
        frame[5] = num_regs & 0xFF;
        
        uint16_t crc = calculate_crc(frame, 6);
        frame[6] = crc & 0xFF;        // CRC Low [cite: 239]
        frame[7] = (crc >> 8) & 0xFF; // CRC High

        write(serial_fd_, frame, 8);
    }

    // Read Response
    bool read_response(std::vector<uint8_t>& buffer, int expected_bytes) {
        if (serial_fd_ < 0) return false;
        
        buffer.resize(expected_bytes);
        int total_read = 0;
        int max_retries = 5; // Simple timeout mechanism
        
        while (total_read < expected_bytes && max_retries > 0) {
            int n = read(serial_fd_, buffer.data() + total_read, expected_bytes - total_read);
            if (n > 0) {
                total_read += n;
            } else {
                usleep(10000); // Wait 10ms
                max_retries--;
            }
        }
        
        if (total_read != expected_bytes) return false;

        // CRC Check
        uint16_t received_crc = buffer[expected_bytes-2] | (buffer[expected_bytes-1] << 8);
        uint16_t calc_crc = calculate_crc(buffer.data(), expected_bytes - 2);
        
        return (received_crc == calc_crc);
    }

    // Helper: Extract 16-bit value (Little Endian from Data Payload) 
    // Data comes as: [ByteL, ByteH]
    uint16_t get_u16(const std::vector<uint8_t>& buf, int offset) {
        return buf[offset] | (buf[offset+1] << 8);
    }

    // Helper: Extract 32-bit value (Little Endian)
    // Data comes as: [Byte0, Byte1, Byte2, Byte3]
    uint32_t get_u32(const std::vector<uint8_t>& buf, int offset) {
        return buf[offset] | (buf[offset+1] << 8) | (buf[offset+2] << 16) | (buf[offset+3] << 24);
    }

    // Helper: Extract 32-bit signed value (Little Endian)
    int32_t get_i32(const std::vector<uint8_t>& buf, int offset) {
        uint32_t val = get_u32(buf, offset);
        return static_cast<int32_t>(val);
    }

    void timer_callback() {
        if (serial_fd_ < 0) {
            if (!open_serial()) return; // Try reconnect
        }

        sensor_msgs::msg::BatteryState msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = frame_id_;
        msg.present = true;

        // --- STEP 1: Read Status Block (0x9000 - 0x900C) ---
        // We read 15 registers (30 bytes data + 5 bytes overhead = 35 bytes)
        // Regs: 9000(WorkState), 9002(Volt), 9004(Curr), 9006-7(CellV), 9008-B(Temp), 900C(Protect)
        send_request(0x9000, 15);
        std::vector<uint8_t> buf1;
        // Expected: 1(Addr)+1(Func)+1(Len)+30(Data)+2(CRC) = 35
        if (read_response(buf1, 35)) {
            // Byte index in data payload starts at 3
            // 0x9000 Work State (2 Bytes) @ idx 3
            uint16_t work_state = get_u16(buf1, 3);
            
            // 0x9002 Voltage (4 Bytes) @ idx 5 -> mV
            uint32_t volt_mv = get_u32(buf1, 5);
            msg.voltage = volt_mv / 1000.0; // mV -> V [cite: 139]

            // 0x9004 Current (4 Bytes) @ idx 9 -> mA
            int32_t curr_ma = get_i32(buf1, 9);
            msg.current = curr_ma / 1000.0; // mA -> A [cite: 139]

            // 0x9008 Max Cell Temp (2 Bytes) @ idx 17
            // "66 means 66-40=26 C" [cite: 139]
            uint16_t temp_raw = get_u16(buf1, 17);
            msg.temperature = temp_raw - 40.0;

            // 0x900C Protect Status (4 Bytes) @ idx 25
            uint32_t protect_status = get_u32(buf1, 25);
            
            // Map Status
            if (protect_status != 0) {
                 msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
                 // Can refine based on bits (Overheat, Overvolt, etc.)
                 if (protect_status & (0x00000008 | 0x00000010)) // Charge Over Temp [cite: 156]
                    msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
            } else {
                 msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
            }

            switch (work_state) {
                case 1: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING; break;
                case 2: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING; break;
                default: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING; break;
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Read Status Block Failed (CRC or Timeout)");
            return; // Don't publish partial data
        }

        // --- STEP 2: Read Capacity Block (0x9028 - 0x902A) ---
        // We read 4 registers (0x9028 SOC, 0x9029 SOH, 0x902A Capacity)
        // Regs: 9028(1), 9029(1), 902A(2) -> Total 4 regs = 8 bytes data
        usleep(20000); // Brief pause between commands
        send_request(0x9028, 4);
        std::vector<uint8_t> buf2;
        // Expected: 1+1+1+8+2 = 13 bytes
        if (read_response(buf2, 13)) {
             // 0x9028 SOC (2 Bytes) @ idx 3
             uint16_t soc = get_u16(buf2, 3);
             msg.percentage = soc / 100.0; // 0-100 -> 0.0-1.0 [cite: 139]

             // 0x902A Capacity (4 Bytes) @ idx 7
             uint32_t cap_mah = get_u32(buf2, 7);
             msg.capacity = cap_mah / 1000.0; // mAh -> Ah
             msg.design_capacity = std::numeric_limits<float>::quiet_NaN(); // Unknown from protocol
        } else {
             RCLCPP_WARN(this->get_logger(), "Read Capacity Block Failed");
        }

        battery_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TwsBmsNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "rclcpp_components/register_node_macro.hpp" 
RCLCPP_COMPONENTS_REGISTER_NODE(TwsBmsNode)