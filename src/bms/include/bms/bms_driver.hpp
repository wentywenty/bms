#ifndef BMS_DRIVER_HPP
#define BMS_DRIVER_HPP

#include <string>
#include <vector>
#include <cstdint>

namespace tws_bms {

struct BatteryStatus {
    double voltage;
    double current;
    double temperature;
    double percentage;
    double charge;      
    double capacity;    
    double design_capacity;
    uint32_t protect_status;
    uint16_t work_state;
    double max_cell_voltage;
    double min_cell_voltage;
    std::string serial_number;
    uint16_t sw_version;
    uint16_t hw_version;
    uint16_t soh;
    uint32_t cycles;
};

class BmsProtocol {
public:
    BmsProtocol(const std::string& port_name, int baud_rate);
    ~BmsProtocol();

    bool open();
    void close_port();
    bool is_open() const { return serial_fd_ >= 0; }

    bool read_basic_info(BatteryStatus& status);
    bool read_version_info(BatteryStatus& status);
    bool read_capacity_info(BatteryStatus& status);
    bool read_serial_number(std::string& sn);
    
    bool set_discharge_output(bool enable);

private:
    int serial_fd_;
    std::string port_name_;
    int baud_rate_;

    uint16_t calculate_crc(const uint8_t *data, size_t len);
    void send_read_request(uint16_t start_addr, uint16_t num_regs);
    void send_write_request(uint16_t reg_addr, uint16_t value);
    bool read_response(std::vector<uint8_t>& buffer, int expected_bytes);
    
    uint16_t get_u16(const std::vector<uint8_t>& buf, int offset);
    uint32_t get_u32(const std::vector<uint8_t>& buf, int offset);
    int32_t get_i32(const std::vector<uint8_t>& buf, int offset);
};

} // namespace tws_bms

#endif // BMS_DRIVER_HPP
