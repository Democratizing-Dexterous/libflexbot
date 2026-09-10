#pragma once

#include "libflexbot/vendor/controlcanfd.hpp"

#include <cstdint>
#include <dlfcn.h>
#include <map>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

namespace libflexbot
{
using namespace vendor;

class CanFD
{
public:
    explicit CanFD(uint32_t device_index = 0,
                   const std::string &serial = "",
                   const std::string &lib_path = "");
    ~CanFD();

    CanFD(const CanFD &) = delete;
    CanFD &operator=(const CanFD &) = delete;

    void init(uint32_t Abit = 1000000, uint32_t Bbit = 5000000);
    void close();
    // can_channel uses the vendor's zero-based index: 1 means physical channel 2.
    void ensure_channel(uint32_t can_channel, bool termination = false);

    bool send_frame(uint32_t can_channel, uint32_t can_id, const std::vector<uint8_t> &data);
    bool send_fd_frame(uint32_t can_channel, uint32_t can_id, const std::vector<uint8_t> &data, bool brs = true);

    struct RxFrame
    {
        uint32_t can_id = 0;
        std::vector<uint8_t> data;
        uint64_t timestamp = 0;
        bool is_fd = false;
    };
    std::vector<RxFrame> receive(uint32_t can_channel, uint32_t max_frames = 256);

    uint32_t device_index() const { return device_index_; }
    std::string serial() const { return serial_; }
    bool initialized() const { return device_ != INVALID_DEVICE_HANDLE; }

private:
    template <typename Fn>
    void load_symbol(Fn &fn, const char *name)
    {
        dlerror();
        void *symbol = dlsym(lib_, name);
        const char *error = dlerror();
        if (error != nullptr || symbol == nullptr)
        {
            throw std::runtime_error(std::string("dlsym failed for ") + name + ": " + (error ? error : "null symbol"));
        }
        fn = reinterpret_cast<Fn>(symbol);
    }

    void load_library();
    void open_device();
    CHANNEL_HANDLE channel_handle(uint32_t can_channel);

    using ZCAN_OpenDeviceFn = DEVICE_HANDLE (*)(UINT, UINT, UINT);
    using ZCAN_CloseDeviceFn = UINT (*)(DEVICE_HANDLE);
    using ZCAN_GetDeviceInfFn = UINT (*)(DEVICE_HANDLE, ZCAN_DEVICE_INFO *);
    using ZCAN_InitCANFn = CHANNEL_HANDLE (*)(DEVICE_HANDLE, UINT, ZCAN_CHANNEL_INIT_CONFIG *);
    using ZCAN_StartCANFn = UINT (*)(CHANNEL_HANDLE);
    using ZCAN_ResetCANFn = UINT (*)(CHANNEL_HANDLE);
    using ZCAN_ClearBufferFn = UINT (*)(CHANNEL_HANDLE);
    using ZCAN_GetReceiveNumFn = UINT (*)(CHANNEL_HANDLE, BYTE);
    using ZCAN_TransmitFn = UINT (*)(CHANNEL_HANDLE, ZCAN_Transmit_Data *, UINT);
    using ZCAN_ReceiveFn = UINT (*)(CHANNEL_HANDLE, ZCAN_Receive_Data *, UINT, int);
    using ZCAN_TransmitFDFn = UINT (*)(CHANNEL_HANDLE, ZCAN_TransmitFD_Data *, UINT);
    using ZCAN_ReceiveFDFn = UINT (*)(CHANNEL_HANDLE, ZCAN_ReceiveFD_Data *, UINT, int);
    using ZCAN_SetAbitBaudFn = UINT (*)(DEVICE_HANDLE, UINT, UINT);
    using ZCAN_SetDbitBaudFn = UINT (*)(DEVICE_HANDLE, UINT, UINT);
    using ZCAN_SetResistanceEnableFn = UINT (*)(DEVICE_HANDLE, UINT, UINT);
    using ZCAN_ClearFilterFn = UINT (*)(CHANNEL_HANDLE);
    using ZCAN_AckFilterFn = UINT (*)(CHANNEL_HANDLE);
    using ZCAN_FindUsbDeviceFn = UINT (*)(ZCAN_DEVICE_INFO *);

    mutable std::mutex mutex_;
    void *lib_ = nullptr;
    DEVICE_HANDLE device_ = INVALID_DEVICE_HANDLE;
    std::map<uint32_t, CHANNEL_HANDLE> channels_;
    std::string lib_path_;
    std::string serial_;
    uint32_t device_index_ = 0;
    uint32_t abit_ = 1000000;
    uint32_t bbit_ = 5000000;

    ZCAN_OpenDeviceFn open_device_fn_ = nullptr;
    ZCAN_CloseDeviceFn close_device_fn_ = nullptr;
    ZCAN_GetDeviceInfFn get_device_inf_fn_ = nullptr;
    ZCAN_InitCANFn init_can_fn_ = nullptr;
    ZCAN_StartCANFn start_can_fn_ = nullptr;
    ZCAN_ResetCANFn reset_can_fn_ = nullptr;
    ZCAN_ClearBufferFn clear_buffer_fn_ = nullptr;
    ZCAN_GetReceiveNumFn get_receive_num_fn_ = nullptr;
    ZCAN_TransmitFn transmit_fn_ = nullptr;
    ZCAN_ReceiveFn receive_fn_ = nullptr;
    ZCAN_TransmitFDFn transmit_fd_fn_ = nullptr;
    ZCAN_ReceiveFDFn receive_fd_fn_ = nullptr;
    ZCAN_SetAbitBaudFn set_abit_baud_fn_ = nullptr;
    ZCAN_SetDbitBaudFn set_dbit_baud_fn_ = nullptr;
    ZCAN_SetResistanceEnableFn set_resistance_enable_fn_ = nullptr;
    ZCAN_ClearFilterFn clear_filter_fn_ = nullptr;
    ZCAN_AckFilterFn ack_filter_fn_ = nullptr;
    ZCAN_FindUsbDeviceFn find_usb_device_fn_ = nullptr;
};

} // namespace libflexbot
