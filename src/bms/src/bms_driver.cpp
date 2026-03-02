#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "bms/bms_driver.hpp"

class TwsBmsNode : public rclcpp::Node {
public:
    explicit TwsBmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("bms_node", options) {
        
        // Parameters
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

        // Initialize Protocol Handler
        bms_ = std::make_unique<tws_bms::BmsProtocol>(port_name, baud_rate);

        if (bms_->open()) {
            RCLCPP_INFO(this->get_logger(), "BMS Serial Port %s opened.", port_name.c_str());
            log_bms_info();
        }

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&TwsBmsNode::timer_callback, this));

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_discharge_output",
            std::bind(&TwsBmsNode::handle_set_output, this, std::placeholders::_1, std::placeholders::_2));
    }

private:
    std::unique_ptr<tws_bms::BmsProtocol> bms_;
    std::string cached_sn_;
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
            RCLCPP_INFO(this->get_logger(), "BMS SW: 0x%04X, HW: 0x%04X, SOH: %d%%, Cycles: %u", 
                status.sw_version, status.hw_version, status.soh, status.cycles);
        }
    }

    void handle_set_output(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!bms_->is_open()) { response->success = false; return; }
        response->success = bms_->set_discharge_output(request->data);
        response->message = response->success ? "Success" : "Failed";
    }

    void check_diagnostics(const tws_bms::BatteryStatus & s) {
        double gap = s.max_cell_voltage - s.min_cell_voltage;
        if (gap > cell_gap_threshold_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, 
                "电芯极差过大: %.3f V", gap);
        }

        if (s.temperature > over_temp_threshold_) {
            RCLCPP_ERROR(this->get_logger(), "!!! 严重高温: %.1f C !!! 执行软件关断", s.temperature);
            bms_->set_discharge_output(false);
        }

        if (s.protect_status != 0) {
            uint32_t p = s.protect_status;
            if (p & 0x00000004) RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "BMS故障: 单体过压");
            if (p & 0x00000040) RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 5000, "BMS故障: 总电压欠压");
        }
    }

    void timer_callback() {
        if (!bms_->is_open() && !bms_->open()) return;

        tws_bms::BatteryStatus s;
        if (bms_->read_basic_info(s)) {
            sensor_msgs::msg::BatteryState msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = this->get_parameter("frame_id").as_string();
            msg.voltage = s.voltage;
            msg.current = s.current;
            msg.temperature = s.temperature;
            
            if (bms_->read_capacity_info(s)) {
                msg.percentage = s.percentage;
                msg.capacity = s.capacity;
                msg.design_capacity = static_cast<float>(design_capacity_);
            }

            if (cached_sn_.empty()) bms_->read_serial_number(cached_sn_);
            msg.serial_number = cached_sn_;
            msg.location = this->get_parameter("location").as_string();
            msg.present = true;
            msg.power_supply_technology = 2; // LION

            check_diagnostics(s);
            battery_pub_->publish(msg);
        } else {
            bms_->close_port();
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwsBmsNode>());
    rclcpp::shutdown();
    return 0;
}
