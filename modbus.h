// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-

#if !defined(MODBUS_H)
#    define MODBUS_H

#    include <Arduino.h> // UART

#    include "channel.h"

#    include <inttypes.h>

template<typename T>
T &&move(T &v)
{
    return static_cast<T &&>(v);
}

namespace modbus
{

typedef uint16_t RegisterID;
typedef uint16_t RegisterValue;
typedef uint8_t NodeAddress;

struct MasterReadMultiple {
    public:
        RegisterID start;
        uint16_t len;
};
struct SlaveReadMultiple {
    public:
        // inline vector<uint16_t, 128>, capacity 128
        uint8_t len;
        RegisterValue values[128];
};

struct MasterPdu {
        typedef enum {
            ReadRegMulti,
        } OpCode;
        OpCode opcode;
        union {
                MasterReadMultiple reg_multi;
        };
        // Something the master sends that isn't inn the modbus struct
        // that appears to be sent back to the master
        // A form of tag maybe?
        uint8_t epilogue_len;
        uint8_t epilogue[10];
};

struct SlavePdu {
        typedef enum {
            ReadRegMulti,
        } OpCode;
        OpCode opcode;
        union {
                SlaveReadMultiple reg_multi;
        };
        uint8_t epilogue_len;
        uint8_t epilogue[10];
};

namespace rtu
{
struct ServerSocket
    : public Receiver<MasterPdu>
    , public Sender<SlavePdu> {
    public:
        ServerSocket(FlushedIO &dev,
                     size_t inter_message_time_min,
                     size_t inter_byte_time_max,
                     NodeAddress addr);
        virtual bool recv(MasterPdu &pdu);
        virtual bool send(SlavePdu &&pdu);

    private:
        FlushedIO &dev_;
        size_t inter_message_time_min_;
        size_t inter_byte_time_max_;
        NodeAddress addr_;
};
} // namespace rtu
}; // namespace modbus

#endif // defined(MODBUS_H)
