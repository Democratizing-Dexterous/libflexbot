#include "libflexbot/canfd.hpp"

#include <algorithm>
#include <cstring>
#include <filesystem>
#include <iostream>

namespace libflexbot
{
namespace
{
constexpr UINT kDeviceType = USBCANFD_200U;
constexpr BYTE kCanFdBrs = 0x01;

std::string default_lib_path()
{
    return std::string(LIBFLEXBOT_PROJECT_ROOT) + "/libs/libcontrolcanfd.so";
}
} // namespace

CanFD::CanFD(uint32_t device_index, const std::string &serial, const std::string &lib_path)
    : lib_path_(lib_path.empty() ? default_lib_path() : lib_path),
      serial_(serial),
      device_index_(device_index)
{
    load_library();
}

CanFD::~CanFD()
{
    close();
}

void CanFD::load_library()
{
    lib_ = dlopen(lib_path_.c_str(), RTLD_NOW | RTLD_LOCAL);
    if (lib_ == nullptr)
    {
        throw std::runtime_error("dlopen failed for " + lib_path_ + ": " + dlerror());
    }

    load_symbol(open_device_fn_, "ZCAN_OpenDevice");
    load_symbol(close_device_fn_, "ZCAN_CloseDevice");
    load_symbol(get_device_inf_fn_, "ZCAN_GetDeviceInf");
    load_symbol(init_can_fn_, "ZCAN_InitCAN");
    load_symbol(start_can_fn_, "ZCAN_StartCAN");
    load_symbol(reset_can_fn_, "ZCAN_ResetCAN");
    load_symbol(clear_buffer_fn_, "ZCAN_ClearBuffer");
    load_symbol(get_receive_num_fn_, "ZCAN_GetReceiveNum");
    load_symbol(transmit_fn_, "ZCAN_Transmit");
    load_symbol(receive_fn_, "ZCAN_Receive");
    load_symbol(transmit_fd_fn_, "ZCAN_TransmitFD");
    load_symbol(receive_fd_fn_, "ZCAN_ReceiveFD");
    load_symbol(set_abit_baud_fn_, "ZCAN_SetAbitBaud");
    load_symbol(set_dbit_baud_fn_, "ZCAN_SetDbitBaud");
    load_symbol(set_resistance_enable_fn_, "ZCAN_SetResistanceEnable");
    load_symbol(clear_filter_fn_, "ZCAN_ClearFilter");
    load_symbol(ack_filter_fn_, "ZCAN_AckFilter");
    load_symbol(find_usb_device_fn_, "ZCAN_FindUsbDevice");
}

void CanFD::open_device()
{
    if (device_ != INVALID_DEVICE_HANDLE)
    {
        return;
    }

    if (!serial_.empty())
    {
        ZCAN_DEVICE_INFO devices[USBCANFD_MAX_NUM]{};
        find_usb_device_fn_(devices);
        bool found = false;
        for (uint32_t i = 0; i < USBCANFD_MAX_NUM; ++i)
        {
            if (devices[i].reserved[0] == 1 &&
                serial_ == reinterpret_cast<const char *>(devices[i].str_Serial_Num))
            {
                device_index_ = i;
                found = true;
                break;
            }
        }
        if (!found)
        {
            throw std::runtime_error("CANFD serial not found: " + serial_);
        }
    }
    device_ = open_device_fn_(kDeviceType, device_index_, 0);
    if (device_ == INVALID_DEVICE_HANDLE)
    {
        throw std::runtime_error("ZCAN_OpenDevice failed");
    }

    ZCAN_DEVICE_INFO info{};
    const UINT ret = get_device_inf_fn_(device_, &info);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_GetDeviceInf failed, ret=" + std::to_string(ret));
    }
    serial_ = reinterpret_cast<const char *>(info.str_Serial_Num);
    std::cout << "CANFD opened: device_index = " << device_index_
              << ", serial = " << serial_
              << ", can_count = " << static_cast<unsigned int>(info.can_Num)
              << ", hw_type = " << info.str_hw_Type << std::endl;
}

void CanFD::init(uint32_t Abit, uint32_t Bbit)
{
    std::lock_guard<std::mutex> lock(mutex_);
    abit_ = Abit;
    bbit_ = Bbit;
    open_device();
}

void CanFD::ensure_channel(uint32_t can_channel, bool termination)
{
    std::lock_guard<std::mutex> lock(mutex_);
    if (device_ == INVALID_DEVICE_HANDLE)
    {
        open_device();
    }
    if (channels_.count(can_channel) != 0)
    {
        return;
    }

    const UINT channel_index = can_channel;
    UINT ret = set_abit_baud_fn_(device_, channel_index, abit_);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_SetAbitBaud failed, ret=" + std::to_string(ret));
    }
    ret = set_dbit_baud_fn_(device_, channel_index, bbit_);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_SetDbitBaud failed, ret=" + std::to_string(ret));
    }
    if (termination)
    {
        ret = set_resistance_enable_fn_(device_, channel_index, 1);
        if (ret != STATUS_OK)
        {
            throw std::runtime_error("ZCAN_SetResistanceEnable failed, ret=" + std::to_string(ret));
        }
    }

    ZCAN_CHANNEL_INIT_CONFIG cfg{};
    cfg.can_type = TYPE_CANFD;
    cfg.canfd.acc_code = 0;
    cfg.canfd.acc_mask = 0xFFFFFFFF;
    cfg.canfd.filter = 1;
    cfg.canfd.mode = 0;
    cfg.canfd.brp = 0;

    CHANNEL_HANDLE channel = init_can_fn_(device_, channel_index, &cfg);
    if (channel == INVALID_CHANNEL_HANDLE)
    {
        throw std::runtime_error("ZCAN_InitCAN failed");
    }
    ret = clear_filter_fn_(channel);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_ClearFilter failed, ret=" + std::to_string(ret));
    }
    ret = ack_filter_fn_(channel);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_AckFilter failed, ret=" + std::to_string(ret));
    }
    ret = start_can_fn_(channel);
    if (ret != STATUS_OK)
    {
        throw std::runtime_error("ZCAN_StartCAN failed, ret=" + std::to_string(ret));
    }
    clear_buffer_fn_(channel);
    channels_[can_channel] = channel;
}

CHANNEL_HANDLE CanFD::channel_handle(uint32_t can_channel)
{
    auto it = channels_.find(can_channel);
    if (it == channels_.end())
    {
        throw std::runtime_error("CAN channel is not initialized: " + std::to_string(can_channel));
    }
    return it->second;
}

bool CanFD::send_frame(uint32_t can_channel, uint32_t can_id, const std::vector<uint8_t> &data)
{
    if (data.size() > CAN_MAX_DLEN)
    {
        throw std::runtime_error("classic CAN payload must be <= 8 bytes");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ZCAN_Transmit_Data tx{};
    tx.frame.can_id = make_can_id(can_id, false, false, false);
    tx.frame.can_dlc = static_cast<BYTE>(data.size());
    tx.transmit_type = 0;
    std::copy(data.begin(), data.end(), tx.frame.data);
    return transmit_fn_(channel_handle(can_channel), &tx, 1) == 1;
}

bool CanFD::send_fd_frame(uint32_t can_channel, uint32_t can_id, const std::vector<uint8_t> &data, bool brs)
{
    if (data.size() > CANFD_MAX_DLEN)
    {
        throw std::runtime_error("CANFD payload must be <= 64 bytes");
    }
    std::lock_guard<std::mutex> lock(mutex_);
    ZCAN_TransmitFD_Data tx{};
    tx.frame.can_id = make_can_id(can_id, false, false, false);
    tx.frame.len = static_cast<BYTE>(data.size());
    tx.frame.flags = brs ? kCanFdBrs : 0;
    tx.transmit_type = 0;
    std::copy(data.begin(), data.end(), tx.frame.data);
    return transmit_fd_fn_(channel_handle(can_channel), &tx, 1) == 1;
}

std::vector<CanFD::RxFrame> CanFD::receive(uint32_t can_channel, uint32_t max_frames)
{
    std::lock_guard<std::mutex> lock(mutex_);
    CHANNEL_HANDLE channel = channel_handle(can_channel);
    std::vector<RxFrame> frames;
    max_frames = std::max<uint32_t>(1, max_frames);

    const UINT can_pending = std::min<UINT>(get_receive_num_fn_(channel, TYPE_CAN), max_frames);
    if (can_pending > 0)
    {
        std::vector<ZCAN_Receive_Data> rx(can_pending);
        const UINT got = receive_fn_(channel, rx.data(), can_pending, 0);
        for (UINT i = 0; i < got; ++i)
        {
            RxFrame f;
            f.can_id = get_id(rx[i].frame.can_id);
            f.timestamp = rx[i].timestamp;
            f.is_fd = false;
            f.data.assign(rx[i].frame.data, rx[i].frame.data + rx[i].frame.can_dlc);
            frames.push_back(std::move(f));
        }
    }

    const UINT remain = max_frames > frames.size() ? max_frames - static_cast<UINT>(frames.size()) : 0;
    const UINT fd_pending = std::min<UINT>(get_receive_num_fn_(channel, TYPE_CANFD), remain);
    if (fd_pending > 0)
    {
        std::vector<ZCAN_ReceiveFD_Data> rx(fd_pending);
        const UINT got = receive_fd_fn_(channel, rx.data(), fd_pending, 0);
        for (UINT i = 0; i < got; ++i)
        {
            RxFrame f;
            f.can_id = get_id(rx[i].frame.can_id);
            f.timestamp = rx[i].timestamp;
            f.is_fd = true;
            f.data.assign(rx[i].frame.data, rx[i].frame.data + rx[i].frame.len);
            frames.push_back(std::move(f));
        }
    }
    return frames;
}

void CanFD::close()
{
    std::lock_guard<std::mutex> lock(mutex_);
    for (auto &entry : channels_)
    {
        if (entry.second != INVALID_CHANNEL_HANDLE && reset_can_fn_ != nullptr)
        {
            reset_can_fn_(entry.second);
        }
    }
    channels_.clear();

    if (device_ != INVALID_DEVICE_HANDLE && close_device_fn_ != nullptr)
    {
        close_device_fn_(device_);
        device_ = INVALID_DEVICE_HANDLE;
    }
    if (lib_ != nullptr)
    {
        dlclose(lib_);
        lib_ = nullptr;
    }
}

} // namespace libflexbot
