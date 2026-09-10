#include "libflexbot/robot.hpp"

#include "libflexbot/frames.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <pthread.h>
#include <sched.h>
#include <sstream>
#include <unistd.h>
#include <yaml-cpp/yaml.h>

namespace libflexbot
{
namespace
{
constexpr uint8_t kClearErrorTail = 0xFB;
constexpr uint8_t kEnableTail = 0xFC;
constexpr uint8_t kDisableTail = 0xFD;
constexpr uint8_t kSetZeroTail = 0xFE;
constexpr auto kFeedbackTimeout = std::chrono::milliseconds(200);
constexpr double kPi = 3.14159265358979323846;

std::string error_state_message(size_t joint_index, uint8_t state)
{
    const size_t joint = joint_index + 1;
    switch (state)
    {
    case 0x8:
        return "第" + std::to_string(joint) + "关节超压";
    case 0x9:
        return "第" + std::to_string(joint) + "关节欠压";
    case 0xA:
        return "第" + std::to_string(joint) + "关节过电流";
    case 0xB:
        return "第" + std::to_string(joint) + "关节MOS过温";
    case 0xC:
        return "第" + std::to_string(joint) + "关节电机线圈过温";
    case 0xD:
        return "第" + std::to_string(joint) + "关节通信丢失";
    case 0xE:
        return "第" + std::to_string(joint) + "关节过载";
    default:
        return "";
    }
}

bool bind_thread_to_cpu(std::thread::native_handle_type thread, int32_t cpu)
{
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(static_cast<int>(cpu), &set);
    return pthread_setaffinity_np(thread, sizeof(set), &set) == 0;
}

uint32_t le32_at(const std::vector<uint8_t> &data, size_t offset)
{
    return static_cast<uint32_t>(data[offset]) |
           (static_cast<uint32_t>(data[offset + 1]) << 8) |
           (static_cast<uint32_t>(data[offset + 2]) << 16) |
           (static_cast<uint32_t>(data[offset + 3]) << 24);
}

// A register reply echoes the motor id, carries 0x00 in data[1] and the
// operation in data[2], which is what tells it apart from a normal feedback
// frame. Reads also echo the register id in data[3]; a save ack leaves it as
// "don't care".
bool is_register_reply(const CanFD::RxFrame &frame, uint32_t id, uint8_t op, uint8_t rid, bool check_rid)
{
    return frame.data.size() >= 4 && (frame.data[0] & 0x0F) == id && frame.data[1] == 0x00 &&
           frame.data[2] == op && (!check_rid || frame.data[3] == rid);
}

double require_double(const YAML::Node &node, const std::string &name)
{
    if (!node[name])
    {
        throw std::runtime_error("missing YAML field: " + name);
    }
    return node[name].as<double>();
}
} // namespace

Robot::Robot(CanFD &canfd,
             uint32_t can_channel,
             uint32_t freq,
             const std::string &config,
             bool soft_limit,
             uint32_t mode)
    : canfd_(canfd),
      can_channel_(can_channel),
      freq_(freq),
      mode_(mode_from_id(mode)),
      soft_limit_(soft_limit)
{
    if (freq_ == 0)
    {
        throw std::runtime_error("freq must be > 0");
    }
    period_ = std::chrono::microseconds(static_cast<int64_t>(1000000 / freq_));
    load_config(config);
    canfd_.ensure_channel(can_channel_);
}

Robot::~Robot()
{
    stop_thread(true);
}

void Robot::load_config(const std::string &path)
{
    YAML::Node root = YAML::LoadFile(path);
    if (!root["motor_types"] || !root["robot_config"] || !root["robot_config"]["motor_configs"])
    {
        throw std::runtime_error("invalid motor config YAML");
    }

    std::unordered_map<std::string, MotorType> motor_types;
    for (const auto &entry : root["motor_types"])
    {
        const std::string name = entry.first.as<std::string>();
        const YAML::Node type_node = entry.second;
        MotorType type;
        type.pmax = require_double(type_node, "pmax");
        type.vmax = require_double(type_node, "vmax");
        type.tmax = require_double(type_node, "tmax");
        motor_types[name] = type;
    }

    for (const YAML::Node &node : root["robot_config"]["motor_configs"])
    {
        MotorConfig motor;
        motor.id = node["id"].as<uint32_t>();
        motor.type = node["type"].as<std::string>();
        auto type_it = motor_types.find(motor.type);
        if (type_it == motor_types.end())
        {
            throw std::runtime_error("unknown motor type: " + motor.type);
        }
        motor.range = type_it->second;
        if (node["limits"] && node["limits"]["position"])
        {
            const YAML::Node pos = node["limits"]["position"];
            // YAML joint limits are degrees; motor protocol positions are radians.
            motor.position_limit.lower = pos["lower"].as<double>() * kPi / 180.0;
            motor.position_limit.upper = pos["upper"].as<double>() * kPi / 180.0;
            motor.position_limit.enabled = true;
        }
        index_by_id_[motor.id] = config_.motors.size();
        config_.motors.push_back(motor);
    }

    if (config_.motors.empty())
    {
        throw std::runtime_error("motor config is empty");
    }
    if (root["robot_config"]["num_motors"] &&
        root["robot_config"]["num_motors"].as<size_t>() != config_.motors.size())
    {
        throw std::runtime_error("robot_config.num_motors does not match motor_configs");
    }
    commands_.assign(config_.motors.size(), MotorCommand{});
    feedback_.assign(config_.motors.size(), MotorFeedback{});
}

void Robot::enable(int32_t cpu)
{
    if (running_.load())
    {
        return;
    }
    if (worker_.joinable())
    {
        worker_.join();
    }
    if (cpu < -1)
    {
        throw std::runtime_error("cpu must be -1 (unbound) or a zero based core index");
    }
    const long cpu_count = sysconf(_SC_NPROCESSORS_ONLN);
    if (cpu >= 0 && cpu_count > 0 && cpu >= cpu_count)
    {
        throw std::runtime_error("cpu " + std::to_string(cpu) + " is out of range: this machine reports " +
                                 std::to_string(cpu_count) + " online cpus");
    }
    set_last_error("");
    stop_requested_.store(false);
    disable_sent_.store(false);
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        feedback_.assign(config_.motors.size(), MotorFeedback{});
    }
    log_event("enable begin");
    try
    {
        send_start_sequence();
        seed_mode_commands();
    }
    catch (const std::exception &ex)
    {
        set_last_error(ex.what());
        log_event(std::string("enable failed: ") + ex.what() + ", disable all motors");
        send_disable_all();
        throw;
    }
    running_.store(true);
    log_event(std::string(mode_name(mode_)) + " control loop started, freq=" +
              std::to_string(freq_) + " Hz");
    worker_ = std::thread(&Robot::worker_loop, this);
    if (cpu >= 0)
    {
        if (!bind_thread_to_cpu(worker_.native_handle(), cpu))
        {
            const std::string reason = "failed to bind control thread to cpu " + std::to_string(cpu);
            set_last_error(reason);
            log_event(reason + ", disable all motors");
            stop_thread(true);
            throw std::runtime_error(reason);
        }
        log_event("control thread bound to cpu " + std::to_string(cpu));
    }
}

void Robot::disable()
{
    log_event("disable requested");
    stop_thread(true);
}

void Robot::stop_thread(bool send_disable)
{
    const bool was_active = running_.load() || worker_.joinable();
    stop_requested_.store(true);
    if (worker_.joinable())
    {
        worker_.join();
    }
    if (send_disable)
    {
        send_disable_all();
    }
    running_.store(false);
    if (send_disable && was_active)
    {
        log_event("control loop stopped");
    }
}

void Robot::send_start_sequence()
{
    for (const MotorConfig &motor : config_.motors)
    {
        const std::string joint = "j" + std::to_string(motor.id);
        try
        {
            if (!send_special(motor.id, kClearErrorTail))
            {
                throw std::runtime_error("frame send failed");
            }
            log_event(joint + " clear_error ok");
        }
        catch (const std::exception &ex)
        {
            log_event(joint + " clear_error failed: " + ex.what());
            throw std::runtime_error("clear error frame send failed for motor " + std::to_string(motor.id));
        }
    }
    for (const MotorConfig &motor : config_.motors)
    {
        const std::string joint = "j" + std::to_string(motor.id);
        try
        {
            const std::string mode = mode_name(mode_);
            if (!canfd_.send_frame(can_channel_, kRegisterFrameId, mode_frame(motor.id, mode_)))
            {
                throw std::runtime_error("frame send failed");
            }
            log_event(joint + " mode " + mode + " ok");
        }
        catch (const std::exception &ex)
        {
            log_event(joint + " mode set failed: " + ex.what());
            throw std::runtime_error("mode frame send failed for motor " + std::to_string(motor.id));
        }
    }
    for (const MotorConfig &motor : config_.motors)
    {
        const std::string joint = "j" + std::to_string(motor.id);
        try
        {
            if (!send_special(motor.id, kEnableTail))
            {
                throw std::runtime_error("enable frame send failed");
            }
            std::string reason;
            if (!wait_for_feedback(motor.id, reason))
            {
                throw std::runtime_error(reason);
            }
            log_event(joint + " enable ok");
        }
        catch (const std::exception &ex)
        {
            log_event(joint + " enable failed: " + ex.what());
            throw std::runtime_error("enable frame send failed for motor " + std::to_string(motor.id));
        }
    }
}

void Robot::seed_mode_commands()
{
    if (mode_ == Mode::Mit)
    {
        return;
    }
    std::vector<std::string> seeded;
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        for (size_t i = 0; i < commands_.size(); ++i)
        {
            const MotorConfig &motor = config_.motors[i];
            const double target = feedback_[i].received_at_steady_us != 0 ? feedback_[i].pos : 0.0;
            if (mode_ == Mode::Pv && !commands_[i].pv.active)
            {
                commands_[i].pv = PvCommand{true, target, 0.0};
                seeded.push_back("j" + std::to_string(motor.id) +
                                 " PV target seeded at measured position " + std::to_string(target));
            }
            else if (mode_ == Mode::Pvt && !commands_[i].pvt.active)
            {
                commands_[i].pvt = PvtCommand{true, target, 0.0, 0.0};
                seeded.push_back("j" + std::to_string(motor.id) +
                                 " PVT target seeded at measured position " + std::to_string(target));
            }
        }
    }
    for (const std::string &message : seeded)
    {
        log_event(message);
    }
}

bool Robot::wait_for_feedback(uint32_t id, std::string &reason)
{
    const auto deadline = std::chrono::steady_clock::now() + kFeedbackTimeout;
    const size_t motor_index = index_by_id_.at(id);

    while (!stop_requested_.load() && std::chrono::steady_clock::now() < deadline)
    {
        const auto frames = canfd_.receive(can_channel_);
        for (const auto &frame : frames)
        {
            if (!update_feedback_from_frame(frame))
            {
                continue;
            }

            std::lock_guard<std::mutex> lock(state_mutex_);
            if (feedback_[motor_index].received_at_steady_us == 0)
            {
                continue;
            }
            const std::string error = error_state_message(motor_index, feedback_[motor_index].state);
            if (!error.empty())
            {
                reason = error;
                return false;
            }
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    reason = "feedback timeout";
    return false;
}

void Robot::send_disable_all()
{
    if (disable_sent_.exchange(true))
    {
        return;
    }
    log_event("disable all begin");
    for (const MotorConfig &motor : config_.motors)
    {
        try
        {
            if (send_special(motor.id, kDisableTail))
            {
                log_event("disable j" + std::to_string(motor.id) + " done");
            }
            else
            {
                log_event("disable j" + std::to_string(motor.id) + " failed");
            }
        }
        catch (const std::exception &ex)
        {
            log_event("disable j" + std::to_string(motor.id) + " failed: " + ex.what());
        }
    }
    log_event("disable all done");
}

bool Robot::send_special(uint32_t id, uint8_t tail)
{
    return canfd_.send_frame(can_channel_, id, {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, tail});
}

bool Robot::control_mit(uint32_t id, double kp, double kd, double p_des, double v_des, double t_ff)
{
    if (mode_ != Mode::Mit)
    {
        log_mode_mismatch("MIT", id);
        return false;
    }
    MitCommand cmd{kp, kd, p_des, v_des, t_ff};
    std::string reason;
    if (!validate_command(id, cmd, reason))
    {
        return reject_command("MIT", id, reason);
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    commands_[index_by_id_.at(id)].mit = cmd;
    return true;
}

bool Robot::control_pv(uint32_t id, double p_des, double v_des)
{
    if (mode_ != Mode::Pv)
    {
        log_mode_mismatch("PV", id);
        return false;
    }
    size_t index = 0;
    std::string reason;
    if (!check_position(id, p_des, index, reason))
    {
        return reject_command("PV", id, reason);
    }
    if (!std::isfinite(v_des))
    {
        return reject_command("PV", id, "v_des is not finite");
    }
    PvCommand cmd;
    cmd.active = true;
    cmd.p_des = p_des;
    cmd.v_des = v_des;
    std::lock_guard<std::mutex> lock(state_mutex_);
    commands_[index].pv = cmd;
    return true;
}

bool Robot::control_pvt(uint32_t id, double p_des, double v_des, double i_des)
{
    if (mode_ != Mode::Pvt)
    {
        log_mode_mismatch("PVT", id);
        return false;
    }
    size_t index = 0;
    std::string reason;
    if (!check_position(id, p_des, index, reason))
    {
        return reject_command("PVT", id, reason);
    }
    if (!std::isfinite(v_des) || !std::isfinite(i_des))
    {
        return reject_command("PVT", id, "v_des or i_des is not finite");
    }
    PvtCommand cmd;
    cmd.active = true;
    cmd.p_des = p_des;
    cmd.v_des = v_des;
    cmd.i_des = i_des;
    std::lock_guard<std::mutex> lock(state_mutex_);
    commands_[index].pvt = cmd;
    return true;
}

bool Robot::set_zero(uint32_t id)
{
    if (index_by_id_.find(id) == index_by_id_.end())
    {
        const std::string reason = "unknown motor id: " + std::to_string(id);
        set_last_error(reason);
        log_event("reject set_zero for motor " + std::to_string(id) + ": " + reason);
        return false;
    }

    const std::string joint = "j" + std::to_string(id);
    try
    {
        if (!send_special(id, kSetZeroTail))
        {
            throw std::runtime_error("frame send failed");
        }
    }
    catch (const std::exception &ex)
    {
        set_last_error(ex.what());
        log_event(joint + " set_zero failed: " + ex.what());
        return false;
    }
    log_event(joint + " set_zero ok");
    return true;
}

// Send a register frame and wait for the matching reply, re-sending every
// `retry` until `timeout` expires. Returns false when the motor stays silent or
// the frame cannot be sent.
bool Robot::exchange_register(uint32_t id,
                              uint16_t code,
                              bool check_rid,
                              bool need_value,
                              uint32_t &value,
                              std::chrono::milliseconds timeout,
                              std::chrono::milliseconds retry)
{
    const std::vector<uint8_t> request = register_frame(id, code);
    const uint8_t op = static_cast<uint8_t>(code & 0xFF);
    const uint8_t rid = static_cast<uint8_t>(code >> 8);

    if (running_.load())
    {
        // The control loop owns the receive path, so it collects the reply.
        {
            std::lock_guard<std::mutex> lock(read_mutex_);
            if (register_op_.pending)
            {
                throw std::runtime_error("another register operation is already in progress");
            }
            register_op_ = RegisterOp{};
            register_op_.pending = true;
            register_op_.id = id;
            register_op_.op = op;
            register_op_.rid = rid;
            register_op_.check_rid = check_rid;
            register_op_.need_value = need_value;
        }
        std::unique_lock<std::mutex> lock(read_mutex_);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        bool timed_out = false;
        while (true)
        {
            // Resend outside the lock so the control loop can keep collecting
            // replies while the frame is on the wire.
            lock.unlock();
            const bool sent = canfd_.send_frame(can_channel_, kRegisterFrameId, request);
            lock.lock();
            if (!sent)
            {
                register_op_.pending = false;
                lock.unlock();
                return false;
            }
            if (read_cv_.wait_for(lock, retry, [this] { return register_op_.done; }))
            {
                break;
            }
            if (std::chrono::steady_clock::now() >= deadline)
            {
                timed_out = true;
                register_op_.pending = false;
                break;
            }
        }
        if (timed_out)
        {
            lock.unlock();
            return false;
        }
        value = register_op_.value;
        register_op_ = RegisterOp{};
        return true;
    }

    const auto deadline = std::chrono::steady_clock::now() + timeout;
    auto next_send = std::chrono::steady_clock::now();
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (std::chrono::steady_clock::now() >= next_send)
        {
            if (!canfd_.send_frame(can_channel_, kRegisterFrameId, request))
            {
                return false;
            }
            next_send = std::chrono::steady_clock::now() + retry;
        }
        for (const CanFD::RxFrame &frame : canfd_.receive(can_channel_))
        {
            if (!is_register_reply(frame, id, op, rid, check_rid))
            {
                continue;
            }
            if (need_value && frame.data.size() < 8)
            {
                continue;
            }
            value = frame.data.size() >= 8 ? le32_at(frame.data, 4) : 0;
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    return false;
}

uint32_t Robot::read_register(uint32_t id, uint16_t reg)
{
    if (index_by_id_.find(id) == index_by_id_.end())
    {
        throw std::runtime_error("unknown motor id: " + std::to_string(id));
    }
    uint32_t value = 0;
    if (!exchange_register(id, reg, true, true, value, kReadTimeout, kReadRetryInterval))
    {
        const std::string reason = "register read timed out for motor " + std::to_string(id);
        set_last_error(reason);
        log_event(reason + " (register " + std::to_string(reg) + ")");
        throw std::runtime_error(reason);
    }
    return value;
}

uint32_t Robot::read_timeout(uint32_t id)
{
    return read_register(id, kTimeoutReadCode);
}

bool Robot::save_register(uint32_t id, uint32_t rid)
{
    if (index_by_id_.find(id) == index_by_id_.end())
    {
        const std::string reason = "unknown motor id: " + std::to_string(id);
        set_last_error(reason);
        log_event("reject register save for motor " + std::to_string(id) + ": " + reason);
        return false;
    }
    if (rid > 0xFF)
    {
        const std::string reason = "register id out of range: " + std::to_string(rid);
        set_last_error(reason);
        log_event("reject register save for motor " + std::to_string(id) + ": " + reason);
        return false;
    }
    const uint16_t code = static_cast<uint16_t>((rid << 8) | kSaveRegisterOp);
    uint32_t value = 0;
    if (!exchange_register(id, code, false, false, value, kSaveTimeout, kSaveRetryInterval))
    {
        const std::string reason = "register save timed out for motor " + std::to_string(id);
        set_last_error(reason);
        log_event(reason + " (register " + std::to_string(rid) + ")");
        return false;
    }
    log_event("j" + std::to_string(id) + " register " + std::to_string(rid) + " saved");
    return true;
}

bool Robot::write_register(uint32_t id, uint16_t reg, uint32_t value)
{
    if (index_by_id_.find(id) == index_by_id_.end())
    {
        const std::string reason = "unknown motor id: " + std::to_string(id);
        set_last_error(reason);
        log_event("reject register write for motor " + std::to_string(id) + ": " + reason);
        return false;
    }
    if (!canfd_.send_frame(can_channel_, kRegisterFrameId, register_frame(id, reg, value)))
    {
        const std::string reason = "register write frame send failed for motor " + std::to_string(id);
        set_last_error(reason);
        log_event(reason);
        return false;
    }
    log_event("j" + std::to_string(id) + " register " + std::to_string(reg) + " set to " +
              std::to_string(value));
    return true;
}

bool Robot::write_timeout(uint32_t id, uint32_t timeout)
{
    return write_register(id, kTimeoutWriteCode, timeout);
}

bool Robot::consume_register_reply(const CanFD::RxFrame &frame)
{
    std::lock_guard<std::mutex> lock(read_mutex_);
    if (!register_op_.pending || register_op_.done)
    {
        return false;
    }
    if (!is_register_reply(frame, register_op_.id, register_op_.op, register_op_.rid, register_op_.check_rid))
    {
        return false;
    }
    if (register_op_.need_value && frame.data.size() < 8)
    {
        return false;
    }
    register_op_.value = frame.data.size() >= 8 ? le32_at(frame.data, 4) : 0;
    register_op_.done = true;
    read_cv_.notify_all();
    return true;
}

bool Robot::check_position(uint32_t id, double p_des, size_t &index, std::string &reason) const
{
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end())
    {
        reason = "unknown motor id: " + std::to_string(id);
        return false;
    }
    index = it->second;
    if (!std::isfinite(p_des))
    {
        reason = "p_des is not finite";
        return false;
    }
    const MotorConfig &motor = config_.motors[index];
    if (soft_limit_ && motor.position_limit.enabled &&
        (p_des < motor.position_limit.lower || p_des > motor.position_limit.upper))
    {
        reason = "p_des out of soft limit";
        return false;
    }
    return true;
}

void Robot::log_mode_mismatch(const char *requested, uint32_t id) const
{
    log_event(std::string("ignore ") + requested + " command for motor " + std::to_string(id) +
              ": robot mode is " + mode_name(mode_) + ", command requires mode=" + requested);
}

bool Robot::reject_command(const char *kind, uint32_t id, const std::string &reason)
{
    set_last_error(reason);
    log_event(std::string("reject ") + kind + " command for motor " + std::to_string(id) + ": " + reason);
    return false;
}

bool Robot::validate_command(uint32_t id, const MitCommand &cmd, std::string &reason) const
{
    size_t index = 0;
    if (!check_position(id, cmd.p_des, index, reason))
    {
        return false;
    }
    const MotorConfig &motor = config_.motors[index];
    auto in_range = [](double x, double lo, double hi) { return std::isfinite(x) && x >= lo && x <= hi; };
    if (!in_range(cmd.p_des, -motor.range.pmax, motor.range.pmax))
    {
        reason = "p_des out of motor range";
        return false;
    }
    if (!in_range(cmd.v_des, -motor.range.vmax, motor.range.vmax))
    {
        reason = "v_des out of motor range";
        return false;
    }
    if (!in_range(cmd.t_ff, -motor.range.tmax, motor.range.tmax))
    {
        reason = "t_ff out of motor range";
        return false;
    }
    if (!in_range(cmd.kp, kp_min_, kp_max_))
    {
        reason = "kp out of range";
        return false;
    }
    if (!in_range(cmd.kd, kd_min_, kd_max_))
    {
        reason = "kd out of range";
        return false;
    }
    return true;
}

int32_t Robot::float_to_uint(double x, double min_value, double max_value, int bits) const
{
    x = std::clamp(x, min_value, max_value);
    const double span = max_value - min_value;
    const double scaled = (x - min_value) * static_cast<double>((1ULL << bits) - 1ULL) / span;
    return static_cast<int32_t>(scaled);
}

double Robot::uint_to_float(uint32_t x, double min_value, double max_value, int bits) const
{
    const double span = max_value - min_value;
    return static_cast<double>(x) * span / static_cast<double>((1ULL << bits) - 1ULL) + min_value;
}

bool Robot::send_command(uint32_t id, const MotorCommand &cmd)
{
    switch (mode_)
    {
    case Mode::Mit:
        return send_mit(id, cmd.mit);
    case Mode::Pv:
        return send_pv(id, cmd.pv);
    case Mode::Pvt:
        return send_pvt(id, cmd.pvt);
    }
    return false;
}

bool Robot::send_mit(uint32_t id, const MitCommand &cmd)
{
    const MotorConfig &motor = config_.motors[index_by_id_.at(id)];
    const uint16_t pos = static_cast<uint16_t>(float_to_uint(cmd.p_des, -motor.range.pmax, motor.range.pmax, 16));
    const uint16_t vel = static_cast<uint16_t>(float_to_uint(cmd.v_des, -motor.range.vmax, motor.range.vmax, 12));
    const uint16_t tor = static_cast<uint16_t>(float_to_uint(cmd.t_ff, -motor.range.tmax, motor.range.tmax, 12));
    const uint16_t kp = static_cast<uint16_t>(float_to_uint(cmd.kp, kp_min_, kp_max_, 12));
    const uint16_t kd = static_cast<uint16_t>(float_to_uint(cmd.kd, kd_min_, kd_max_, 12));

    std::vector<uint8_t> data(8);
    data[0] = static_cast<uint8_t>(pos >> 8);
    data[1] = static_cast<uint8_t>(pos);
    data[2] = static_cast<uint8_t>(vel >> 4);
    data[3] = static_cast<uint8_t>(((vel & 0xF) << 4) | (kp >> 8));
    data[4] = static_cast<uint8_t>(kp);
    data[5] = static_cast<uint8_t>(kd >> 4);
    data[6] = static_cast<uint8_t>(((kd & 0xF) << 4) | (tor >> 8));
    data[7] = static_cast<uint8_t>(tor);

    return canfd_.send_frame(can_channel_, id + kMitIdOffset, data);
}

bool Robot::send_pv(uint32_t id, const PvCommand &cmd)
{
    return canfd_.send_frame(can_channel_, id + kPvIdOffset, pv_frame(cmd.p_des, cmd.v_des));
}

bool Robot::send_pvt(uint32_t id, const PvtCommand &cmd)
{
    return canfd_.send_frame(can_channel_, id + kPvtIdOffset,
                             pvt_frame(cmd.p_des, cmd.v_des, cmd.i_des));
}

bool Robot::update_feedback_from_frame(const CanFD::RxFrame &frame)
{
    if (frame.data.size() < 8)
    {
        return false;
    }
    const uint32_t id = frame.data[0] & 0x0F;
    auto it = index_by_id_.find(id);
    if (it == index_by_id_.end())
    {
        return false;
    }
    const size_t index = it->second;
    const MotorConfig &motor = config_.motors[index];

    const uint32_t p_int = (static_cast<uint32_t>(frame.data[1]) << 8) | frame.data[2];
    const uint32_t v_int = (static_cast<uint32_t>(frame.data[3]) << 4) | (frame.data[4] >> 4);
    const uint32_t t_int = ((static_cast<uint32_t>(frame.data[4]) & 0x0F) << 8) | frame.data[5];

    MotorFeedback fb;
    fb.state = frame.data[0] >> 4;
    fb.pos = uint_to_float(p_int, -motor.range.pmax, motor.range.pmax, 16);
    fb.vel = uint_to_float(v_int, -motor.range.vmax, motor.range.vmax, 12);
    fb.tau = uint_to_float(t_int, -motor.range.tmax, motor.range.tmax, 12);
    fb.t_mos = frame.data[6];
    fb.t_rotor = frame.data[7];
    fb.timestamp = frame.timestamp;
    fb.received_at_steady_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());

    std::lock_guard<std::mutex> lock(state_mutex_);
    feedback_[index] = fb;
    return true;
}

bool Robot::check_errors(std::string &reason) const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < feedback_.size(); ++i)
    {
        if (feedback_[i].received_at_steady_us == 0)
        {
            continue;
        }
        const std::string message = error_state_message(i, feedback_[i].state);
        if (!message.empty())
        {
            reason = message;
            return false;
        }
    }
    return true;
}

bool Robot::check_soft_limits(std::string &reason) const
{
    if (!soft_limit_)
    {
        return true;
    }
    std::lock_guard<std::mutex> lock(state_mutex_);
    for (size_t i = 0; i < feedback_.size(); ++i)
    {
        const MotorConfig &motor = config_.motors[i];
        if (feedback_[i].received_at_steady_us == 0 || !motor.position_limit.enabled)
        {
            continue;
        }
        if (feedback_[i].pos < motor.position_limit.lower || feedback_[i].pos > motor.position_limit.upper)
        {
            reason = "第" + std::to_string(i + 1) + "关节超出软限位";
            return false;
        }
    }
    return true;
}

void Robot::worker_loop()
{
    try
    {
        // Keep the receive path usable if a previous wrapper instance closed the channel.
        canfd_.ensure_channel(can_channel_);
        auto next = std::chrono::steady_clock::now();
        auto last_feedback = next;
        while (!stop_requested_.load())
        {
            const auto frames = canfd_.receive(can_channel_);
            bool parsed_feedback = false;
            for (const auto &frame : frames)
            {
                if (consume_register_reply(frame))
                {
                    continue;
                }
                parsed_feedback = update_feedback_from_frame(frame) || parsed_feedback;
            }
            if (parsed_feedback)
            {
                last_feedback = std::chrono::steady_clock::now();
            }

            if (std::chrono::steady_clock::now() - last_feedback > kFeedbackTimeout)
            {
                set_last_error("CAN feedback timeout");
                log_event("CAN feedback timeout, disable all motors and exit control loop");
                send_disable_all();
                break;
            }

            std::string reason;
            if (!check_errors(reason) || !check_soft_limits(reason))
            {
                set_last_error(reason);
                log_event(reason + ", disable all motors and exit control loop");
                send_disable_all();
                break;
            }

            std::vector<MotorCommand> snapshot;
            {
                std::lock_guard<std::mutex> lock(state_mutex_);
                snapshot = commands_;
            }
            for (size_t i = 0; i < config_.motors.size(); ++i)
            {
                if (!send_command(config_.motors[i].id, snapshot[i]))
                {
                    const std::string failure = std::string(mode_name(mode_)) + " frame send failed";
                    set_last_error(failure);
                    log_event(failure + ", disable all motors and exit control loop");
                    send_disable_all();
                    stop_requested_.store(true);
                    break;
                }
            }

            next += period_;
            std::this_thread::sleep_until(next);
        }
    }
    catch (const std::exception &ex)
    {
        set_last_error(ex.what());
        log_event(std::string("control loop exception: ") + ex.what() +
                  ", disable all motors and exit control loop");
        send_disable_all();
    }
    stop_requested_.store(true);
    running_.store(false);
    log_event("control loop exited");
}

std::vector<MotorFeedback> Robot::get_feedback() const
{
    std::lock_guard<std::mutex> lock(state_mutex_);
    std::vector<MotorFeedback> result = feedback_;
    const uint64_t now_us = static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
    for (MotorFeedback &fb : result)
    {
        if (fb.received_at_steady_us == 0 || now_us < fb.received_at_steady_us)
        {
            fb.delay_ms = -1.0;
        }
        else
        {
            fb.delay_ms = static_cast<double>(now_us - fb.received_at_steady_us) / 1000.0;
        }
    }
    return result;
}

std::vector<uint32_t> Robot::motor_ids() const
{
    std::vector<uint32_t> ids;
    ids.reserve(config_.motors.size());
    for (const MotorConfig &motor : config_.motors)
    {
        ids.push_back(motor.id);
    }
    return ids;
}

std::string Robot::last_error() const
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    return last_error_;
}

void Robot::set_last_error(const std::string &message)
{
    std::lock_guard<std::mutex> lock(error_mutex_);
    last_error_ = message;
}

void Robot::log_event(const std::string &message) const
{
    std::lock_guard<std::mutex> lock(log_mutex_);
    const char *log_dir_env = std::getenv("LIBFLEXBOT_LOG_DIR");
    const std::filesystem::path log_dir = log_dir_env != nullptr
                                               ? std::filesystem::path(log_dir_env)
                                               : std::filesystem::current_path() / "logs";
    const std::filesystem::path path = log_dir / "robot_events.txt";
    std::filesystem::create_directories(path.parent_path());

    const std::time_t now = std::time(nullptr);
    std::tm tm{};
    localtime_r(&now, &tm);
    std::ostringstream line;
    line << std::put_time(&tm, "%F %T") << " " << message;

    std::ofstream out(path, std::ios::app);
    if (out)
    {
        out << line.str() << "\n";
        out.flush();
    }
    std::cout << line.str() << std::endl;
}

} // namespace libflexbot
