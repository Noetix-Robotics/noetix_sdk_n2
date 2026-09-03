#include "highcontroller.h"
#include <aolion_driver.h>
#include <pybind11/cast.h>
#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

namespace py = pybind11;
using namespace noetix;

PYBIND11_MODULE(highcontrol_py, m) {
        // ControlCmd
        py::enum_<ControlCmd>(m, "ControlCmd")
            .value("WALK", ControlCmd::WALK)
            .value("SWING", ControlCmd::SWING)
            .value("SHAKE", ControlCmd::SHAKE)
            .value("CHEER", ControlCmd::CHEER)
            .value("RUN", ControlCmd::RUN)
            .value("START", ControlCmd::START)
            .value("SWITCH", ControlCmd::SWITCH)
            .value("STARTTEACH", ControlCmd::STARTTEACH)
            .value("SAVETEACH", ControlCmd::SAVETEACH)
            .value("ENDTEACH", ControlCmd::ENDTEACH)
            .value("PLAYTEACH", ControlCmd::PLAYTEACH)
            .value("DANCE1", ControlCmd::DANCE1)
            .value("DANCE2", ControlCmd::DANCE2)
            .value("DANCE3", ControlCmd::DANCE3)
            .value("DANCE4", ControlCmd::DANCE4)
            .value("DANCE5", ControlCmd::DANCE5)
            .value("CF", ControlCmd::CF)
            .value("TANG", ControlCmd::TANG)
            .value("DEFAULT", ControlCmd::DEFAULT)
            .export_values();

        m.doc() = "HighController Python binding";

        py::class_<joydata>(m, "JoyData")
            .def(py::init<>())
            .def_property_readonly("axes",
                                   [](const joydata &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)}, d.axes);
                                   })
            .def_property_readonly("button", [](const joydata &d) {
                    return py::array_t<int>({14}, {sizeof(int)}, d.button);
            });

        py::class_<NingImuData>(m, "NingImuData")
            .def(py::init<>())
            .def_property_readonly("ori",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {4}, {sizeof(double)}, d.ori);
                                   })
            .def_property_readonly("ori_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.ori_cov);
                                   })
            .def_property_readonly("angular_vel",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.angular_vel);
                                   })
            .def_property_readonly("angular_vel_cov",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {9}, {sizeof(double)},
                                               d.angular_vel_cov);
                                   })
            .def_property_readonly("linear_acc",
                                   [](const NingImuData &d) {
                                           return py::array_t<double>(
                                               {3}, {sizeof(double)},
                                               d.linear_acc);
                                   })
            .def_property_readonly("linear_acc_cov", [](const NingImuData &d) {
                    return py::array_t<double>({9}, {sizeof(double)},
                                               d.linear_acc_cov);
            });

        py::class_<MotorState>(m, "MotorState")
            .def(py::init<>())
            .def_readwrite("pos", &MotorState::pos)
            .def_readwrite("vel", &MotorState::vel)
            .def_readwrite("tau", &MotorState::tau)
            .def_readwrite("motor_id", &MotorState::motor_id)
            .def_readwrite("error", &MotorState::error)
            .def_readwrite("temperature", &MotorState::temperature);

        py::class_<HighController>(m, "HighController")
            .def_static("instance", &HighController::Instance,
                        py::return_value_policy::reference)

            .def("init", &HighController::init)

            .def("publish_cmd", &HighController::publish_cmd, py::arg("x"),
                 py::arg("y"), py::arg("yaw"), py::arg("action"),
                 py::arg("index") = 0)

            .def("subscribe_robot_hardware_status",
                 [](HighController &self, py::function callback) {
                         self.subscribe_robot_hardware_status(
                             [callback](const RobotHardwareStatus &status) {
                                     py::gil_scoped_acquire acquire;
                                     callback(py::cast(status));
                             });
                 });



        py::class_<AoLionDriver>(m, "AoLionDriver")
            .def(py::init<>())

            // init
            .def(
                "init",
                [](AoLionDriver &self, const std::string &port, int baudrate) {
                        return self.init(port, baudrate);
                },
                py::arg("port"), py::arg("baudrate"))

            // getremotedata
            .def("getremotedata", [](AoLionDriver &self) {
                    joydata d = self.getremotedata();
                    return d; // 直接返回，依赖你已经绑定 JoyData
            });
}
