#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include "bms/bms_driver.hpp"

using namespace std::chrono_literals;

class TwsBmsNode : public rclcpp::Node {
public:
    explicit TwsBmsNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
        : Node("bms_node", options), consecutive_failures_(0) {
        
        // --- Parameters ---
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
        } else {
            RCLCPP_ERROR(this->get_logger(), "\033[1;31m[错误] 无法打开串口 %s\033[0m", port_name.c_str());
        }

        battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("battery_state", 10);
        timer_ = this->create_wall_timer(
            std::chrono::duration<double>(1.0 / rate),
            std::bind(&TwsBmsNode::timer_callback, this));

        service_ = this->create_service<std_srvs::srv::SetBool>(
            "set_discharge_output",
            std::bind(&TwsBmsNode::handle_set_output, this, std::placeholders::_1, std::placeholders::_2));
            
        RCLCPP_INFO(this->get_logger(), "BMS 驱动已就绪.");
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
            RCLCPP_INFO(this->get_logger(), "BMS 固件: 0x%04X | SOH: %d%% | 循环: %u", status.sw_version, status.soh, status.cycles);
        }
    }

    void handle_set_output(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                           std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        if (!bms_->is_open()) { response->success = false; return; }
        response->success = bms_->set_discharge_output(request->data);
        response->message = response->success ? "Confirmed" : "BMS Rejected";
    }

    void check_diagnostics(const tws_bms::BatteryStatus & s) {
        double gap = s.max_cell_voltage - s.min_cell_voltage;
        if (gap > cell_gap_threshold_) {
            RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 10000, "\033[1;33m[警告] 电芯极差大: %.3fV\033[0m", gap);
        }
        if (s.temperature > over_temp_threshold_) {
            RCLCPP_ERROR(this->get_logger(), "高温紧急关断！");
            bms_->set_discharge_output(false);
        }
    }

    void timer_callback() {
        if (!bms_->is_open()) {
            RCLCPP_INFO(this->get_logger(), "尝试重新打开串口...");
            if (!bms_->open()) return;
            RCLCPP_INFO(this->get_logger(), "串口已恢复连接");
            consecutive_failures_ = 0;
        }

        tws_bms::BatteryStatus s;
        // 尝试读取。
        bool success = bms_->read_basic_info(s) && bms_->read_capacity_info(s);

        if (success) {
            consecutive_failures_ = 0; // 读取成功，清空失败计数
            
            sensor_msgs::msg::BatteryState msg;
            msg.header.stamp = this->now();
            msg.header.frame_id = this->get_parameter("frame_id").as_string();
            msg.voltage = s.voltage;
            msg.current = s.current;
            msg.temperature = s.temperature;
            msg.percentage = s.percentage;
            msg.capacity = s.capacity;
            msg.design_capacity = static_cast<float>(design_capacity_);
            msg.present = true;
            
            if (cached_sn_.empty()) bms_->read_serial_number(cached_sn_);
            msg.serial_number = cached_sn_;

            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 5000, 
                "\033[1;32m[运行中] %.2fV | %.2fA | %.1f%%\033[0m", s.voltage, s.current, s.percentage * 100.0);

            check_diagnostics(s);
            battery_pub_->publish(msg);
        } else {
            consecutive_failures_++;
            RCLCPP_WARN(this->get_logger(), "\033[1;33m[跳过] 帧读取失败 (%d/5)\033[0m", consecutive_failures_);
            
            // 如果连续失败超过 5 次，强制重启串口
            if (consecutive_failures_ >= 5) {
                RCLCPP_ERROR(this->get_logger(), "\033[1;31m连续多次失败，触发串口软重置...\033[0m");
                bms_->close_port(); 
            }
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TwsBmsNode>());
    rclcpp::shutdown();
    return 0;
}
