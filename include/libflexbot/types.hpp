#pragma once

#include <cstdint>
#include <stdexcept>
#include <string>
#include <vector>

namespace libflexbot
{

struct MotorType
{
    double pmax = 0.0;
    double vmax = 0.0;
    double tmax = 0.0;
};

struct PositionLimit
{
    double lower = 0.0;
    double upper = 0.0;
    bool enabled = false;
};

struct MotorConfig
{
    uint32_t id = 0;
    std::string type;
    MotorType range;
    PositionLimit position_limit;
};

struct MotorFeedback
{
    double pos = 0.0;
    double vel = 0.0;
    double tau = 0.0;
    double t_mos = 0.0;
    double t_rotor = 0.0;
    uint64_t timestamp = 0;
    double delay_ms = -1.0;
    uint8_t state = 0;
    uint64_t received_at_steady_us = 0;
};

// Motor-control modes and their register values.
enum class Mode : uint32_t
{
    Mit = 1,
    Pv = 2,
    Pvt = 4,
};

inline const char *mode_name(Mode mode)
{
    switch (mode)
    {
    case Mode::Mit:
        return "MIT";
    case Mode::Pv:
        return "PV";
    case Mode::Pvt:
        return "PVT";
    }
    return "unknown";
}

inline Mode mode_from_id(uint32_t value)
{
    switch (value)
    {
    case static_cast<uint32_t>(Mode::Mit):
        return Mode::Mit;
    case static_cast<uint32_t>(Mode::Pv):
        return Mode::Pv;
    case static_cast<uint32_t>(Mode::Pvt):
        return Mode::Pvt;
    }
    throw std::runtime_error("mode must be 1 (mit), 2 (pv) or 4 (pvt)");
}

struct MitCommand
{
    double kp = 0.0;
    double kd = 0.0;
    double p_des = 0.0;
    double v_des = 0.0;
    double t_ff = 0.0;
};

struct PvCommand
{
    bool active = false;
    double p_des = 0.0;
    double v_des = 0.0;
};

struct PvtCommand
{
    bool active = false;
    double p_des = 0.0;
    double v_des = 0.0;
    double i_des = 0.0;
};

// Command set of a single motor; the control loop sends the one that matches the
// configured mode. The control loop sends Pv/Pvt commands every tick, and enable()
// seeds an inactive Pv/Pvt command with the measured position, so a robot that was
// enabled before any target was set holds its position instead of driving to zero.
struct MotorCommand
{
    MitCommand mit;
    PvCommand pv;
    PvtCommand pvt;
};

struct RobotConfig
{
    std::vector<MotorConfig> motors;
};

} // namespace libflexbot
