#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include "bms/bms_driver.hpp"

namespace py = pybind11;

PYBIND11_MODULE(tws_bms_api, m) {
    m.doc() = "TWS BMS Python API (Refactored)";

    py::class_<tws_bms::BatteryStatus>(m, "BatteryStatus")
        .def(py::init<>())
        .def_readwrite("voltage", &tws_bms::BatteryStatus::voltage)
        .def_readwrite("current", &tws_bms::BatteryStatus::current)
        .def_readwrite("temperature", &tws_bms::BatteryStatus::temperature)
        .def_readwrite("percentage", &tws_bms::BatteryStatus::percentage)
        .def_readwrite("charge", &tws_bms::BatteryStatus::charge)
        .def_readwrite("capacity", &tws_bms::BatteryStatus::capacity)
        .def_readwrite("design_capacity", &tws_bms::BatteryStatus::design_capacity)
        .def_readwrite("protect_status", &tws_bms::BatteryStatus::protect_status)
        .def_readwrite("work_state", &tws_bms::BatteryStatus::work_state)
        .def_readwrite("max_cell_voltage", &tws_bms::BatteryStatus::max_cell_voltage)
        .def_readwrite("min_cell_voltage", &tws_bms::BatteryStatus::min_cell_voltage)
        .def_readwrite("serial_number", &tws_bms::BatteryStatus::serial_number)
        .def_readwrite("sw_version", &tws_bms::BatteryStatus::sw_version)
        .def_readwrite("hw_version", &tws_bms::BatteryStatus::hw_version)
        .def_readwrite("soh", &tws_bms::BatteryStatus::soh)
        .def_readwrite("cycles", &tws_bms::BatteryStatus::cycles);

    py::class_<tws_bms::BmsProtocol>(m, "BmsProtocol")
        .def(py::init<const std::string&, int>())
        .def("open", &tws_bms::BmsProtocol::open)
        .def("close_port", &tws_bms::BmsProtocol::close_port)
        .def("is_open", &tws_bms::BmsProtocol::is_open)
        .def("read_basic_info", &tws_bms::BmsProtocol::read_basic_info)
        .def("read_version_info", &tws_bms::BmsProtocol::read_version_info)
        .def("read_capacity_info", &tws_bms::BmsProtocol::read_capacity_info)
        .def("read_serial_number", [](tws_bms::BmsProtocol &self) {
            std::string sn;
            if (self.read_serial_number(sn)) return sn;
            return std::string("");
        })
        .def("set_discharge_output", &tws_bms::BmsProtocol::set_discharge_output);
}
