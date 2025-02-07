#include <memory>
#include <vector>

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "src/communication/odrive_socket.h"
#include "src/controller/low_level_controller.h"
#include "src/controller/low_level_types.h"


namespace py = pybind11;
using namespace pybind11::literals;


std::shared_ptr<ODriveSocket> create_odrive_socket(std::string if_name) {
    return std::make_shared<ODriveSocket>(if_name);
}


PYBIND11_MODULE(lowlevelapi, m) {
    m.doc() = "Low-level Control API bindings";

    m.def("create_odrive_socket", &create_odrive_socket, "if_name"_a);

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

    py::class_<ODriveSocket>(m, "ODriveSocket")
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

    py::class_<lowleveltypes::MotorCommand>(m, "MotorCommand")
        .def(py::init<>())
        .def_readwrite("position_setpoint", &lowleveltypes::MotorCommand::position_setpoint)
        .def_readwrite("velocity_setpoint", &lowleveltypes::MotorCommand::velocity_setpoint)
        .def_readwrite("torque_feedforward", &lowleveltypes::MotorCommand::torque_feedforward)
        .def_readwrite("damping", &lowleveltypes::MotorCommand::damping)
        .def_readwrite("stiffness", &lowleveltypes::MotorCommand::stiffness)
        .def_readwrite("kp", &lowleveltypes::MotorCommand::kp)
        .def_readwrite("kd", &lowleveltypes::MotorCommand::kd);

    py::class_<lowleveltypes::MotorState>(m, "MotorState")
        .def(py::init<>())
        .def_readwrite("position", &lowleveltypes::MotorState::position)
        .def_readwrite("velocity", &lowleveltypes::MotorState::velocity)
        .def_readwrite("torque_estimate", &lowleveltypes::MotorState::torque_estimate)
        .def_readwrite("current_setpoint", &lowleveltypes::MotorState::current_setpoint)
        .def_readwrite("current_measured", &lowleveltypes::MotorState::current_measured)
        .def(py::pickle(
            [](const lowleveltypes::MotorState &obj) {
                return py::make_tuple(obj.position, obj.velocity, obj.torque_estimate, obj.current_setpoint, obj.current_measured);
            },
            [](py::tuple t) {
                if (t.size() != 5)
                    throw std::runtime_error("Invalid state object");
                lowleveltypes::MotorState obj;
                obj.position = t[0].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.velocity = t[1].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.torque_estimate = t[2].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.current_setpoint = t[3].cast<std::array<float, lowleveltypes::num_motors>>();
                obj.current_measured = t[4].cast<std::array<float, lowleveltypes::num_motors>>();
                return obj;
            }
        ))
        .def("__copy__", [](const lowleveltypes::MotorState& obj) {
            return lowleveltypes::MotorState(obj);
        })
        .def("__deepcopy__", [](const lowleveltypes::MotorState& obj, py::dict) {
            return lowleveltypes::MotorState(obj);
        }, "memo"_a);
        
    py::class_<MotorController>(m, "MotorController")
        .def(py::init<std::shared_ptr<ODriveSocket>, std::vector<canid_t>>(), "odrv"_a, "motor_ids"_a)
        .def("set_axis_state", &MotorController::set_axis_state, "axis_state"_a)
        .def("set_control_mode", &MotorController::set_control_mode, "control_mode"_a, "input_mode"_a = ODriveInputMode::PASSTHROUGH)
        .def("initialize_control_thread", &MotorController::initialize_control_thread)
        .def("stop_control_thread", &MotorController::stop_control_thread)
        .def("update_command", &MotorController::update_command, "command"_a)
        .def("get_motor_states", &MotorController::get_motor_states);
    
}