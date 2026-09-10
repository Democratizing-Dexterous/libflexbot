#pragma once

#include "libflexbot/canfd.hpp"
#include "libflexbot/types.hpp"

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <unordered_map>
#include <vector>

namespace libflexbot
{

class Robot
{
public:
    // mode is one of the Mode register values: 1 (mit), 2 (pv) or 4 (pvt).
    Robot(CanFD &canfd,
          uint32_t can_channel,
          uint32_t freq,
          const std::string &config,
          bool soft_limit = true,
          uint32_t mode = static_cast<uint32_t>(Mode::Mit));
    ~Robot();

    Robot(const Robot &) = delete;
    Robot &operator=(const Robot &) = delete;

    // cpu pins the control thread to one zero based core, -1 leaves it unbound.
    void enable(int32_t cpu = -1);
    void disable();
    // Each control_* call stores the target for the running control loop and
    // fails when the robot was not configured for that mode.
    bool control_mit(uint32_t id, double kp, double kd, double p_des, double v_des, double t_ff);
    bool control_pv(uint32_t id, double p_des, double v_des);
    bool control_pvt(uint32_t id, double p_des, double v_des, double i_des);
    bool set_zero(uint32_t id);
    // Read a motor register: the value the motor answers with. Throws when the
    // id is unknown or when the motor does not answer within kReadTimeout.
    uint32_t read_register(uint32_t id, uint16_t reg);
    uint32_t read_timeout(uint32_t id);
    // Write a motor register. Returns false and sets last_error on failure.
    bool write_register(uint32_t id, uint16_t reg, uint32_t value);
    bool write_timeout(uint32_t id, uint32_t timeout);
    // Ask the motor to persist a register (operation 0xAA) and wait for its
    // ack. rid 0 saves every register; returns false and sets last_error when
    // the motor does not answer within kSaveTimeout.
    bool save_register(uint32_t id, uint32_t rid = 0);
    std::vector<MotorFeedback> get_feedback() const;
    std::vector<uint32_t> motor_ids() const;

    bool running() const { return running_.load(); }
    uint32_t mode() const { return static_cast<uint32_t>(mode_); }
    std::string last_error() const;

private:
    void load_config(const std::string &path);
    void worker_loop();
    bool update_feedback_from_frame(const CanFD::RxFrame &frame);
    bool wait_for_feedback(uint32_t id, std::string &reason, std::chrono::milliseconds window);
    bool check_errors(std::string &reason) const;
    bool check_soft_limits(std::string &reason) const;
    void send_start_sequence();
    void drain_receive();
    void seed_mode_commands();
    void send_disable_all();
    bool send_special(uint32_t id, uint8_t tail);
    bool send_command(uint32_t id, const MotorCommand &cmd);
    bool send_mit(uint32_t id, const MitCommand &cmd);
    bool send_pv(uint32_t id, const PvCommand &cmd);
    bool send_pvt(uint32_t id, const PvtCommand &cmd);
    bool consume_register_reply(const CanFD::RxFrame &frame);
    bool exchange_register(uint32_t id,
                           uint16_t code,
                           bool check_rid,
                           bool need_value,
                           uint32_t &value,
                           std::chrono::milliseconds timeout,
                           std::chrono::milliseconds retry);
    bool check_position(uint32_t id, double p_des, size_t &index, std::string &reason) const;
    bool validate_command(uint32_t id, const MitCommand &cmd, std::string &reason) const;
    void log_mode_mismatch(const char *requested, uint32_t id) const;
    bool reject_command(const char *kind, uint32_t id, const std::string &reason);
    int32_t float_to_uint(double x, double min_value, double max_value, int bits) const;
    double uint_to_float(uint32_t x, double min_value, double max_value, int bits) const;
    void stop_thread(bool send_disable);
    void set_last_error(const std::string &message);
    void log_event(const std::string &message) const;

    // A register reply arrives within a few milliseconds, even while the control
    // loop keeps running. The request is re-sent every kReadRetryInterval until
    // it is answered or kReadTimeout expires, so one dropped request or reply
    // does not fail the read.
    static constexpr std::chrono::milliseconds kReadTimeout{100};
    static constexpr std::chrono::milliseconds kReadRetryInterval{20};
    // Saving writes flash and the motor ack can take up to 30 ms, so it gets a
    // wider window and a slower retry than a plain register read.
    static constexpr std::chrono::milliseconds kSaveTimeout{200};
    static constexpr std::chrono::milliseconds kSaveRetryInterval{50};

    static constexpr double kp_min_ = 0.0;
    static constexpr double kp_max_ = 500.0;
    static constexpr double kd_min_ = 0.0;
    static constexpr double kd_max_ = 5.0;

    CanFD &canfd_;
    uint32_t can_channel_ = 1;
    uint32_t freq_ = 1000;
    Mode mode_ = Mode::Mit;
    bool soft_limit_ = true;
    std::chrono::microseconds period_{1000};

    RobotConfig config_;
    std::unordered_map<uint32_t, size_t> index_by_id_;
    mutable std::mutex state_mutex_;
    std::vector<MotorCommand> commands_;
    std::vector<MotorFeedback> feedback_;
    mutable std::mutex error_mutex_;
    std::string last_error_;
    mutable std::mutex log_mutex_;
    // Register operations are answered from the control loop when it is
    // running, so the reply is never mistaken for a feedback frame. code packs
    // the operation in its low byte and the register id in its high byte.
    struct RegisterOp
    {
        bool pending = false;
        bool done = false;
        uint32_t id = 0;
        uint8_t op = 0;
        uint8_t rid = 0;
        bool check_rid = true;
        bool need_value = false;
        uint32_t value = 0;
    };
    std::mutex read_mutex_;
    std::condition_variable read_cv_;
    RegisterOp register_op_;
    std::atomic_bool stop_requested_{false};
    std::atomic_bool running_{false};
    std::atomic_bool disable_sent_{true};
    std::thread worker_;
};

} // namespace libflexbot
