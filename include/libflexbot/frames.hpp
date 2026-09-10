#pragma once

#include "libflexbot/types.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <vector>

namespace libflexbot
{

// CAN identifiers are the motor id plus a per-mode offset.
constexpr uint32_t kMitIdOffset = 0x000;
constexpr uint32_t kPvIdOffset = 0x100;
constexpr uint32_t kPvtIdOffset = 0x300;

// Register frames are sent to the broadcast id and laid out as
// [id, 0x00, operation, register, value (LE32)]: 0x33 asks the motor to report a
// register, 0x55 asks it to store one. A read sends value 0 and the motor
// answers with the same frame carrying the value. The two bytes are kept as one
// little endian code here, so the codes below read as [operation][register].
constexpr uint32_t kRegisterFrameId = 0x7FF;
constexpr uint16_t kModeWriteCode = 0x0A55;
constexpr uint16_t kTimeoutReadCode = 0x0933;
constexpr uint16_t kTimeoutWriteCode = 0x0955;
// 0xAA asks the motor to persist a register to flash. The frame is laid out
// like a normal register frame, [id, 0x00, 0xAA, rid], and the motor answers
// with an ack whose data[2] is 0xAA again. RID 0 saves every register, a
// specific RID saves just that one.
constexpr uint8_t kSaveRegisterOp = 0xAA;
constexpr uint32_t kPvtMaxScale = 10000;

inline void put_float_le(std::vector<uint8_t> &out, double value)
{
    const float f = static_cast<float>(value);
    uint32_t bits = 0;
    std::memcpy(&bits, &f, sizeof(bits));
    out.push_back(static_cast<uint8_t>(bits & 0xFF));
    out.push_back(static_cast<uint8_t>((bits >> 8) & 0xFF));
    out.push_back(static_cast<uint8_t>((bits >> 16) & 0xFF));
    out.push_back(static_cast<uint8_t>((bits >> 24) & 0xFF));
}

inline void put_uint16_le(std::vector<uint8_t> &out, uint32_t value)
{
    out.push_back(static_cast<uint8_t>(value & 0xFF));
    out.push_back(static_cast<uint8_t>((value >> 8) & 0xFF));
}

// PVT velocity and current are scaled unsigned integers that saturate at 10000.
inline uint32_t scale_pvt(double value, double factor)
{
    if (!std::isfinite(value))
    {
        return 0;
    }
    const double scaled = std::trunc(value * factor);
    if (scaled <= 0.0)
    {
        return 0;
    }
    if (scaled >= static_cast<double>(kPvtMaxScale))
    {
        return kPvtMaxScale;
    }
    return static_cast<uint32_t>(scaled);
}

// PV control frame: float32 position followed by float32 speed limit, both rad
// based. Sent to id + 0x100.
inline std::vector<uint8_t> pv_frame(double p_des, double v_des)
{
    std::vector<uint8_t> data;
    data.reserve(8);
    put_float_le(data, p_des);
    put_float_le(data, v_des);
    return data;
}

// PVT control frame: float32 position, speed limit scaled by 100 and current
// limit scaled by 10000. Sent to id + 0x300.
inline std::vector<uint8_t> pvt_frame(double p_des, double v_des, double i_des)
{
    std::vector<uint8_t> data;
    data.reserve(8);
    put_float_le(data, p_des);
    put_uint16_le(data, scale_pvt(v_des, 100.0));
    put_uint16_le(data, scale_pvt(i_des, 10000.0));
    return data;
}

inline std::vector<uint8_t> register_frame(uint32_t id, uint16_t reg, uint32_t value = 0)
{
    return {static_cast<uint8_t>(id),
            0x00,
            static_cast<uint8_t>(reg & 0xFF),
            static_cast<uint8_t>(reg >> 8),
            static_cast<uint8_t>(value & 0xFF),
            static_cast<uint8_t>((value >> 8) & 0xFF),
            static_cast<uint8_t>((value >> 16) & 0xFF),
            static_cast<uint8_t>((value >> 24) & 0xFF)};
}

inline std::vector<uint8_t> mode_frame(uint32_t id, Mode mode)
{
    return register_frame(id, kModeWriteCode, static_cast<uint32_t>(mode));
}

} // namespace libflexbot
