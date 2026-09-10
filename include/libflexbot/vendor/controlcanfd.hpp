#pragma once

#include <cstdint>
#include <cstddef>

namespace libflexbot::vendor
{

using UINT = uint32_t;
using BYTE = uint8_t;
using USHORT = uint16_t;
using UINT64 = uint64_t;
using DEVICE_HANDLE = void *;
using CHANNEL_HANDLE = void *;

constexpr UINT TYPE_CAN = 0;
constexpr UINT TYPE_CANFD = 1;
constexpr UINT USBCANFD_200U = 41;
constexpr UINT STATUS_OK = 1;
constexpr std::nullptr_t INVALID_DEVICE_HANDLE = nullptr;
constexpr std::nullptr_t INVALID_CHANNEL_HANDLE = nullptr;
constexpr UINT CAN_MAX_DLEN = 8;
constexpr UINT CANFD_MAX_DLEN = 64;
constexpr UINT USBCANFD_MAX_NUM = 8;

constexpr UINT CAN_EFF_FLAG = 0x80000000U;
constexpr UINT CAN_RTR_FLAG = 0x40000000U;
constexpr UINT CAN_ERR_FLAG = 0x20000000U;
constexpr UINT CAN_ID_FLAG = 0x1FFFFFFFU;

constexpr UINT make_can_id(UINT id, bool eff, bool rtr, bool err)
{
    return id | (static_cast<UINT>(eff) << 31) |
           (static_cast<UINT>(rtr) << 30) |
           (static_cast<UINT>(err) << 29);
}

constexpr UINT get_id(UINT id)
{
    return id & CAN_ID_FLAG;
}

#pragma pack(push, 1)

struct can_frame
{
    UINT can_id;
    BYTE can_dlc;
    BYTE __pad;
    BYTE __res0;
    BYTE __res1;
    BYTE data[CAN_MAX_DLEN];
};

struct canfd_frame
{
    UINT can_id;
    BYTE len;
    BYTE flags;
    BYTE __res0;
    BYTE __res1;
    BYTE data[CANFD_MAX_DLEN];
};

struct ZCAN_DEVICE_INFO
{
    USHORT hw_Version;
    USHORT fw_Version;
    USHORT dr_Version;
    USHORT in_Version;
    USHORT irq_Num;
    BYTE can_Num;
    BYTE str_Serial_Num[21];
    BYTE str_hw_Type[40];
    USHORT reserved[4];
};

struct ZCAN_CHANNEL_INIT_CONFIG
{
    UINT can_type;
    union
    {
        struct
        {
            UINT acc_code;
            UINT acc_mask;
            UINT reserved;
            BYTE filter;
            BYTE timing0;
            BYTE timing1;
            BYTE mode;
        } can;
        struct
        {
            UINT acc_code;
            UINT acc_mask;
            UINT abit_timing;
            UINT dbit_timing;
            UINT brp;
            BYTE filter;
            BYTE mode;
            USHORT pad;
            UINT reserved;
        } canfd;
    };
};

struct ZCAN_Transmit_Data
{
    can_frame frame;
    UINT transmit_type;
};

struct ZCAN_Receive_Data
{
    can_frame frame;
    UINT64 timestamp;
};

struct ZCAN_TransmitFD_Data
{
    canfd_frame frame;
    UINT transmit_type;
};

struct ZCAN_ReceiveFD_Data
{
    canfd_frame frame;
    UINT64 timestamp;
};

#pragma pack(pop)

} // namespace libflexbot::vendor
