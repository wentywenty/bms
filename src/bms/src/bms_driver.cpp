#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "bms/bms_driver.hpp"

using namespace std::chrono_literals;

class TwsBmsNode : public rclcpp::Node {
public:
    explicit TwsBmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("bms_node", options), consecutive_failures_(0) {
        
        this->declare_parameter("port_name", "/dev/ttyUSB0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("publish_rate", 1.0);
        this->declare_parameter("frame_id", "battery_link");
        this->declare_parameter("location", "base_link");
        this->declare_parameter("design_capacity", 2.6);
        this->declare_parameter("over_temp_threshold", 55.0);
        this->declare_parameter("low_soc_warn_threshold", 0.15);
        this->declare_parameter("cell_gap_warn_threshold", 0.1);

        std::string port_name = this->get_parameter("port_name").as_string();
        int baud_rate = this->get_parameter("baud_rate").as_int();
        double rate = this->get_parameter("publish_rate").as_double();
        design_capacity_ = this->get_parameter("design_capacity").as_double();
        over_temp_threshold_ = this->get_parameter("over_temp_threshold").as_double();
        low_soc_threshold_ = this->get_parameter("low_soc_warn_threshold").as_double();
        cell_gap_threshold_ = this->get_parameter("cell_gap_warn_threshold").as_double();

        bms_ = std::make_unique<tws_bms::BmsProtocol>(port_name, baud_rate);

        if (bms_->open()) {
            RCLCPP_INFO(this->get_logger(), "\033[1;32m[初始化] 串口 %s 已打开\033[0m", port_name.c_str());
            log_bms_info();
        }

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        timer_ = this->create_wall_timer(std::chrono::duration<double>(1.0 / rate), std::bind(&TwsBmsNode::timer_callback, this));
        service_ = this->create_service<std_srvs::srv::SetBool>("set_discharge_output", std::bind(&TwsBmsNode::handle_set_output, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::unique_ptr<tws_bms::BmsProtocol> bms_;
    std::string cached_sn_;
    int consecutive_failures_;
    double design_capacity_;
    double over_temp_threshold_;
    double low_soc_threshold_;
    double cell_gap_threshold_;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr service_;

    void log_bms_info() {
        tws_bms::BatteryStatus status;
        if (bms_->read_version_info(status)) {
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
            RCLCPP_INFO(this->get_logger(), "硬件版本: 0x%04X | 软件版本: 0x%04X", status.hw_version, status.sw_version);
            RCLCPP_INFO(this->get_logger(), "健康度 (SOH): %d%% | 循环次数: %u", status.soh, status.cycles);
            RCLCPP_INFO(this->get_logger(), "----------------------------------------");
        }
    }

    void handle_set_output(const std::shared_ptr<std_srvs::srv::SetBool::Request> request, std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!bms_->is_open()) { response->success = false; return; }
        response->success = bms_->set_discharge_output(request->data);
        response->message = response->success ? "Confirmed" : "BMS Rejected";
    }

    void check_diagnostics(sensor_msgs::msg::BatteryState & msg, uint32_t protect_status) {
        // 1. Cell Gap
        double gap = msg.cell_voltage.size() >= 2 ? (msg.cell_voltage[0] - msg.cell_voltage[1]) : 0.0;
        if (gap > cell_gap_threshold_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "\033[1;33m[警告] 电芯极差大: %.3fV\033[0m", gap);
        }

        // 2. Secondary Over-temp
        if (msg.temperature > over_temp_threshold_) {
            RCLCPP_ERROR(this->get_logger(), "!!! 严重高温: %.1fC !!! 紧急关断", msg.temperature);
            bms_->set_discharge_output(false);
            msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
        }

        // 3. Health & Status Mapping
        if (protect_status == 0) {
            msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
        } else {
            msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
            if (protect_status & 0x00000004) msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
            if (protect_status & 0x00000040) msg.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
        }
    }

    void timer_callback() {
        if (!bms_->is_open() && !bms_->open()) return;

        tws_bms::BatteryStatus s;
        if (bms_->read_basic_info(s) && bms_->read_capacity_info(s)) {
            consecutive_failures_ = 0;
            sensor_msgs::msg::BatteryState msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = this->get_parameter("frame_id").as_string();
            msg.voltage = s.voltage;
            msg.current = s.current;
            msg.temperature = s.temperature;
            msg.percentage = s.percentage;
            msg.charge = s.charge;
            msg.capacity = s.capacity;
            msg.design_capacity = static_cast<float>(design_capacity_);
            
            // Map Work State to ROS Status
            switch (s.work_state) {
                case 1: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING; break;
                case 2: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING; break;
                case 4: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL; break;
                default: msg.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING; break;
            }

            if (cached_sn_.empty()) bms_->read_serial_number(cached_sn_);
            msg.serial_number = cached_sn_;
            msg.location = this->get_parameter("location").as_string();
            msg.present = true;
            msg.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LION;

            check_diagnostics(msg, s.protect_status);
            battery_pub_->publish(msg);

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "\033[1;32m[运行中] %.2fV | %.2fA | SOC: %.1f%% | 剩余: %.2fAh\033[0m", 
                msg.voltage, msg.current, msg.percentage * 100.0, msg.charge);
        } else {
            consecutive_failures_++;
            if (consecutive_failures_ >= 5) bms_->close_port();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwsBmsNode>());
    rclcpp::shutdown();
    return 0;
}
