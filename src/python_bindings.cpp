#include "libflexbot/canfd.hpp"
#include "libflexbot/robot.hpp"

#include <boost/noncopyable.hpp>
#include <boost/python.hpp>
#include <boost/python/numpy.hpp>
#include <memory>

namespace py = boost::python;
namespace np = boost::python::numpy;

namespace
{

template <typename T>
np::ndarray make_array(const std::vector<T> &values)
{
    np::dtype dtype = np::dtype::get_builtin<T>();
    np::ndarray array = np::zeros(py::make_tuple(values.size()), dtype);
    T *data = reinterpret_cast<T *>(array.get_data());
    std::copy(values.begin(), values.end(), data);
    return array;
}

py::dict robot_getj(const libflexbot::Robot &robot)
{
    const std::vector<libflexbot::MotorFeedback> feedback = robot.get_feedback();
    std::vector<double> pos;
    std::vector<double> vel;
    std::vector<double> tau;
    std::vector<double> t_mos;
    std::vector<double> t_rotor;
    std::vector<unsigned long long> timestamp;
    std::vector<int> state;
    std::vector<double> delay;

    pos.reserve(feedback.size());
    vel.reserve(feedback.size());
    tau.reserve(feedback.size());
    t_mos.reserve(feedback.size());
    t_rotor.reserve(feedback.size());
    timestamp.reserve(feedback.size());
    state.reserve(feedback.size());
    delay.reserve(feedback.size());

    for (const auto &fb : feedback)
    {
        pos.push_back(fb.pos);
        vel.push_back(fb.vel);
        tau.push_back(fb.tau);
        t_mos.push_back(fb.t_mos);
        t_rotor.push_back(fb.t_rotor);
        timestamp.push_back(static_cast<unsigned long long>(fb.timestamp));
        state.push_back(static_cast<int>(fb.state));
        delay.push_back(fb.delay_ms);
    }

    py::dict result;
    result["pos"] = make_array(pos);
    result["vel"] = make_array(vel);
    result["tau"] = make_array(tau);
    result["t_mos"] = make_array(t_mos);
    result["t_rotor"] = make_array(t_rotor);
    result["timestamp"] = make_array(timestamp);
    result["state"] = make_array(state);
    result["delay"] = make_array(delay);
    return result;
}

py::list robot_motor_ids(const libflexbot::Robot &robot)
{
    py::list ids;
    for (uint32_t id : robot.motor_ids())
    {
        ids.append(id);
    }
    return ids;
}

} // namespace

BOOST_PYTHON_MODULE(_libflexbot)
{
    np::initialize();

    py::class_<libflexbot::CanFD, boost::noncopyable>(
        "_CanFD",
        py::init<uint32_t, std::string, std::string>(
            (py::arg("device_index") = 0, py::arg("serial") = "", py::arg("lib_path") = "")))
        .def("init", &libflexbot::CanFD::init, (py::arg("Abit") = 1000000, py::arg("Bbit") = 5000000))
        .def("close", &libflexbot::CanFD::close)
        .def("ensure_channel", &libflexbot::CanFD::ensure_channel,
             (py::arg("can_channel"), py::arg("termination") = false))
        .def("device_index", &libflexbot::CanFD::device_index)
        .def("serial", &libflexbot::CanFD::serial)
        .def("initialized", &libflexbot::CanFD::initialized);

    py::class_<libflexbot::Robot, boost::noncopyable>(
        "_Robot",
        py::init<libflexbot::CanFD &, uint32_t, uint32_t, std::string, bool, uint32_t>(
            (py::arg("canfd"),
             py::arg("can_channel"),
             py::arg("freq"),
             py::arg("config"),
             py::arg("soft_limit") = true,
             py::arg("mode") = 1)))
        .def("enable", &libflexbot::Robot::enable, (py::arg("cpu") = -1))
        .def("disable", &libflexbot::Robot::disable)
        .def("control_mit", &libflexbot::Robot::control_mit,
             (py::arg("id"), py::arg("kp"), py::arg("kd"), py::arg("p_des"), py::arg("v_des"), py::arg("t_ff")))
        .def("control_pv", &libflexbot::Robot::control_pv,
             (py::arg("id"), py::arg("p_des"), py::arg("v_des")))
        .def("control_pvt", &libflexbot::Robot::control_pvt,
             (py::arg("id"), py::arg("p_des"), py::arg("v_des"), py::arg("i_des")))
        .def("set_zero", &libflexbot::Robot::set_zero, (py::arg("id")))
        .def("read_register", &libflexbot::Robot::read_register,
             (py::arg("id"), py::arg("register")))
        .def("read_timeout", &libflexbot::Robot::read_timeout, (py::arg("id")))
        .def("write_register", &libflexbot::Robot::write_register,
             (py::arg("id"), py::arg("register"), py::arg("value")))
        .def("write_timeout", &libflexbot::Robot::write_timeout,
             (py::arg("id"), py::arg("timeout")))
        .def("save_register", &libflexbot::Robot::save_register,
             (py::arg("id"), py::arg("rid") = 0))
        .def("getj", &robot_getj)
        .def("motor_ids", &robot_motor_ids)
        .def("mode", &libflexbot::Robot::mode)
        .def("running", &libflexbot::Robot::running)
        .def("last_error", &libflexbot::Robot::last_error);
}
