#include <filesystem>
#include <memory>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/stl/filesystem.h>

#include "odrive-api/communication/odrive_socket.h"
#include "odrive-api/interface/unitree_driver.h"
#include "odrive-api/containers.h"


namespace py = pybind11;
using namespace pybind11::literals;
using namespace odrive::containers;


PYBIND11_MODULE(odrive_api, m) {
    m.doc() = "Low-level Control API bindings for the ODrive motor controllers";

    py::enum_<ODriveCanID>(m, "ODriveCanID")
        .value("HEARTBEAT", ODriveCanID::HEARTBEAT)
        .value("GET_ENCODER_ESTIMATES", ODriveCanID::GET_ENCODER_ESTIMATES)
        .value("GET_IQ", ODriveCanID::GET_IQ)
        .value("GET_TEMPERATURE", ODriveCanID::GET_TEMPERATURE)
        .value("GET_BUS_VOLTAGE_CURRENT", ODriveCanID::GET_BUS_VOLTAGE_CURRENT)
        .value("GET_TORQUES", ODriveCanID::GET_TORQUES)
        .value("SET_AXIS_STATE", ODriveCanID::SET_AXIS_STATE)
        .value("SET_CONTROL_MODE", ODriveCanID::SET_CONTROL_MODE)
        .value("SET_LIMITS", ODriveCanID::SET_LIMITS)
        .value("SET_POS_GAIN", ODriveCanID::SET_POS_GAIN)
        .value("SET_VEL_GAINS", ODriveCanID::SET_VEL_GAINS)
        .value("SET_POSITION", ODriveCanID::SET_POSITION)
        .value("SET_VELOCITY", ODriveCanID::SET_VELOCITY)
        .value("SET_TORQUE", ODriveCanID::SET_TORQUE)
        .value("CLEAR_ERRORS", ODriveCanID::CLEAR_ERRORS);
    
    py::enum_<ODriveAxisState>(m, "ODriveAxisState")
        .value("UNDEFINED", ODriveAxisState::UNDEFINED)
        .value("IDLE", ODriveAxisState::IDLE)
        .value("CLOSED_LOOP_CONTROL", ODriveAxisState::CLOSED_LOOP_CONTROL);

    py::enum_<ODriveControlMode>(m, "ODriveControlMode")
        .value("VOLTAGE", ODriveControlMode::VOLTAGE)
        .value("TORQUE", ODriveControlMode::TORQUE)
        .value("VELOCITY", ODriveControlMode::VELOCITY)
        .value("POSITION", ODriveControlMode::POSITION);

    py::enum_<ODriveInputMode>(m, "ODriveInputMode")
        .value("INACTIVE", ODriveInputMode::INACTIVE)
        .value("PASSTHROUGH", ODriveInputMode::PASSTHROUGH)
        .value("VEL_RAMP", ODriveInputMode::VEL_RAMP)
        .value("POS_FILTER", ODriveInputMode::POS_FILTER)
        .value("TRAP_TRAJ", ODriveInputMode::TRAP_TRAJ)
        .value("TORQUE_RAMP", ODriveInputMode::TORQUE_RAMP);

    py::class_<ODriveSocket, std::shared_ptr<ODriveSocket>>(m, "ODriveSocket")
        .def(py::init<std::string>(), "if_name"_a)
        .def("getAxisError", &ODriveSocket::getAxisError, "id"_a)
        .def("getAxisState", &ODriveSocket::getAxisState, "id"_a)
        .def("getPositionEstimate", &ODriveSocket::getPositionEstimate, "id"_a)
        .def("getVelocityEstimate", &ODriveSocket::getVelocityEstimate, "id"_a)
        .def("getTorqueEstimate", &ODriveSocket::getTorqueEstimate, "id"_a)
        .def("getIqSetpoint", &ODriveSocket::getIqSetpoint, "id"_a)
        .def("getIqMeasured", &ODriveSocket::getIqMeasured, "id"_a)
        .def("getFETTemperature", &ODriveSocket::getFETTemperature, "id"_a)
        .def("getMotorTemperature", &ODriveSocket::getMotorTemperature, "id"_a)
        .def("getBusVoltage", &ODriveSocket::getBusVoltage, "id"_a)
        .def("getBusCurrent", &ODriveSocket::getBusCurrent, "id"_a)
        .def("setAxisState", &ODriveSocket::setAxisState, "id"_a, "axis_state"_a)
        .def("setControlMode", &ODriveSocket::setControlMode, "id"_a, "control_mode"_a, "input_mode"_a = ODriveInputMode::PASSTHROUGH)
        .def("position_command", &ODriveSocket::position_command, "id"_a, "pos_setpoint"_a, "vel_feedforward"_a = 0.F, "torq_feedforward"_a = 0.F)
        .def("velocity_command", &ODriveSocket::velocity_command, "id"_a, "vel_setpoint"_a, "torq_feedforward"_a = 0.F)
        .def("torque_command", &ODriveSocket::torque_command, "id"_a, "torq_setpoint"_a)
        .def("setLimits", &ODriveSocket::setLimits, "id"_a, "vel_limit"_a, "curr_limit"_a)
        .def("set_stiffness", &ODriveSocket::set_stiffness, "id"_a, "pos_gain"_a)
        .def("set_damping", &ODriveSocket::set_damping, "id"_a, "vel_gain"_a, "vel_integrator_gain"_a = 0.0f)
        .def("clearErrors", &ODriveSocket::clearErrors, "id"_a);

    py::class_<MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("position_setpoint", &MotorCommand::position_setpoint)
        .def_readwrite("velocity_setpoint", &MotorCommand::velocity_setpoint)
        .def_readwrite("torque_feedforward", &MotorCommand::torque_feedforward)
        .def_readwrite("damping", &MotorCommand::damping)
        .def_readwrite("velocity_integrator", &MotorCommand::velocity_integrator)
        .def_readwrite("stiffness", &MotorCommand::stiffness);

    py::class_<MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("position", &MotorState::position)
        .def_readwrite("velocity", &MotorState::velocity)
        .def_readwrite("torque_estimate", &MotorState::torque_estimate)
        .def_readwrite("current_setpoint", &MotorState::current_setpoint)
        .def_readwrite("current_measured", &MotorState::current_measured)
        .def(py::pickle(
            [](const MotorState &obj) {
                return py::make_tuple(obj.position, obj.velocity, obj.torque_estimate, obj.current_setpoint, obj.current_measured);
            },
            [](py::tuple t) {
                if (t.size() != 5)
                    throw std::runtime_error("Invalid state object");
                MotorState obj;
                obj.position = t[0].cast<std::array<float, num_motors>>();
                obj.velocity = t[1].cast<std::array<float, num_motors>>();
                obj.torque_estimate = t[2].cast<std::array<float, num_motors>>();
                obj.current_setpoint = t[3].cast<std::array<float, num_motors>>();
                obj.current_measured = t[4].cast<std::array<float, num_motors>>();
                return obj;
            }
        ))
        .def("__copy__", [](const MotorState& obj) {
            return MotorState(obj);
        })
        .def("__deepcopy__", [](const MotorState& obj, py::dict) {
            return MotorState(obj);
        }, "memo"_a);

        py::class_<LogData>(m, "LogData")
        .def(py::init<>())
        .def_readwite("position", &LogData::position)
        .def_readwite("velocity", &LogData::velocity)
        .def_readwite("torque_estimate", &LogData::torque_estimate)
        .def_readwite("current_setpoint", &LogData::current_setpoint)
        .def_readwite("current_measured", &LogData::current_measured)
        .def_readwite("fet_temperature", &LogData::fet_temperature)
        .def(py::pickle(
            [](const LogData &obj) {
                return py::make_tuple(obj.position, obj.velocity, obj.torque_estimate, obj.current_setpoint, obj.current_measured, obj.fet_temperature);
            },
            [](py::tuple t) {
                if (t.size() != 6)
                    throw std::runtime_error("Invalid state object");
                LogData obj;
                obj.position = t[0].cast<std::array<float, num_motors>>();
                obj.velocity = t[1].cast<std::array<float, num_motors>>();
                obj.torque_estimate = t[2].cast<std::array<float, num_motors>>();
                obj.current_setpoint = t[3].cast<std::array<float, num_motors>>();
                obj.current_measured = t[4].cast<std::array<float, num_motors>>();
                obj.fet_temperature = t[5].cast<std::array<float, num_motors>>();
                return obj;
            }
        ))
        .def("__copy__", [](const LogData& obj) {
            return LogData(obj);
        })
        .def("__deepcopy__", [](const LogData& obj, py::dict) {
            return LogData(obj);
        }, "memo"_a);

    py::class_<Logger>(m, "Logger")
        .def(py::init<std::shared_ptr<ODriveSocket>, std::vector<canid_t>,  std::filesystem::path, int>(), "odrv"_a, "motor_ids"_a, "filepath"_a, "log_rate_us"_a = 2000)
        .def("initialize", &Logger::initialize)
        .def("initialize_thread", &Logger::initialize_thread)
        .def("stop_thread", &Logger::stop_thread);
        
    py::class_<ODriveDriver>(m, "ODriveDriver")
        .def(py::init<std::shared_ptr<ODriveSocket>, std::vector<canid_t>, int>(), "odrv"_a, "motor_ids"_a, "control_rate_us"_a = 2000)
        .def("set_axis_state", &ODriveDriver::set_axis_state, "axis_state"_a)
        .def("set_control_mode", &ODriveDriver::set_control_mode, "control_mode"_a, "input_mode"_a = ODriveInputMode::PASSTHROUGH)
        .def("initialize_thread", &ODriveDriver::initialize_thread)
        .def("stop_thread", &ODriveDriver::stop_thread)
        .def("update_command", &ODriveDriver::update_command, "command"_a)
        .def("get_motor_states", &ODriveDriver::get_motor_states)
        .def("get_axis_state", &ODriveDriver::get_axis_state);
    
}