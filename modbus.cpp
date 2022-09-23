// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-
#include "modbus.h"
#include "panic.h"

#include "crc16.h"
#include "raw_le.h"
#include "raw_be.h"

#include <Arduino.h>

#define CONSOLE SerialUSB

namespace modbus
{
namespace rtu
{
ServerSocket::ServerSocket(FlushedIO &dev,
                           size_t inter_message_time_min,
                           size_t inter_byte_time_max,
                           NodeAddress addr)
    : dev_(dev)
    , inter_message_time_min_(inter_message_time_min)
    , inter_byte_time_max_(inter_byte_time_max)
    , addr_(addr)
{
}

// addr(1)+opcode(1)+payload(1..257)+crc(2)+epilogue;
uint8_t rxbuf[1 + 1 + 257 + 2 + 10];
uint16_t rxbuf_len;

bool wait_readready_with_timeout(FlushedIO &dev, unsigned long duration)
{
    unsigned long b = micros();
    while (true) {
        unsigned long n = micros();
        if ((n - b) > duration) { return false; }
        if (dev.in_waiting() != 0) { return true; }
        delayMicroseconds(1);
    }
}

void wait_readready(FlushedIO &dev)
{
    while (true) {
        if (dev.in_waiting() != 0) { break; }
        delayMicroseconds(1);
    }
}

bool ServerSocket::recv(MasterPdu &pdu)
{
    enum State {
        Start,
        Addr,
        Opcode,
        Payload,
        CRC,
        Epilogue,
        Done,
    };

    NodeAddress addr = 0;

    uint8_t buf[sizeof(uint16_t) + sizeof(uint16_t)];
    uint16_t buf_len = 0;

    CRC16 crc;

    pdu.epilogue_len = 0;

    State state = State::Start;
    while (state != State::Done) {
        uint8_t v;
        int rc;
        switch (state) {
            case State::Start: {
                // wait for quiet, must be 3.5 chars worth of quiet for a start.
                dev_.flush_input();
                if (wait_readready_with_timeout(dev_, inter_message_time_min_)) {
                    // data rx'd waiting for start of frame.. start over..
                    state = State::Start;
                } else {
                    // no data,  so starting frame.
                    state = State::Addr;
                }
            } break;
            case State::Addr: {
                // wait for data, this is the only point to wait longer than 1.5 chars after the
                // start
                wait_readready(dev_);
                rc = dev_.read(v);
                // continue to read the frame, even if not for us, so to retain sync
                addr = v;
                crc.reset();
                crc.update(v);
                state = State::Opcode;
            } break;
            case State::Opcode: {
                if (!wait_readready_with_timeout(dev_, inter_byte_time_max_)) {
                    // no data rx'd in time between bytes
                    state = State::Start;
                } else {
                    rc = dev_.read(v);
                    crc.update(v);
                    switch (v) {
                        case 3:
                            pdu.opcode = MasterPdu::ReadRegMulti;
                            pdu.reg_multi.len = 0;
                            break;
                        default:
                            panic0();
                    }
                    buf_len = 0;
                    state = State::Payload;
                }
            } break;
            case State::Payload: {
                if (!wait_readready_with_timeout(dev_, inter_byte_time_max_)) {
                    // no data rx'd in time between bytes
                    state = State::Start;
                } else {
                    rc = dev_.read(v);
                    crc.update(v);
                    switch (pdu.opcode) {
                        case MasterPdu::ReadRegMulti: {
                            if (buf_len >= sizeof(buf)) { panic0(); }
                            buf[buf_len] = v;
                            buf_len += 1;
                            if (buf_len >= (2 + 2)) {
                                pdu.reg_multi.start = raw::be::decode(&buf[0]);
                                pdu.reg_multi.len = raw::be::decode(&buf[2]);
                                buf_len = 0;
                                state = State::CRC;
                            }
                            break;
                            default:
                                panic0();
                        }
                    }
                }
                break;
                case State::CRC: {
                    if (!wait_readready_with_timeout(dev_, inter_byte_time_max_)) {
                        // no data rx'd in time between bytes
                        state = State::Start;
                    } else {
                        rc = dev_.read(v);
                        if (buf_len >= sizeof(buf)) { panic0(); }
                        buf[buf_len] = v;
                        buf_len += 1;
                        if (buf_len >= 2) {
                            uint16_t crc2 = raw::le::decode(&buf[0]);
                            CONSOLE.print("crc=");
                            CONSOLE.print(crc2, HEX);
                            CONSOLE.print("==");
                            CONSOLE.print(crc.value(), HEX);
                            if (crc2 != crc.value()) {
                                state = State::Start;
                            } else if (addr == 0 || addr == addr_) {
                                // addr==0: broadcast frame, allow through
                                // addr==addr_: unicast for this node, allow through
                                pdu.epilogue_len = 0;
                                state = State::Epilogue;
                            } else {
                                // not for this socket
                                state = State::Start;
                            }
                        }
                    }
                } break;
                case State::Epilogue: {
                    if (!wait_readready_with_timeout(dev_, inter_message_time_min_)) {
                        // no data rx'd - good end of frame
                        state = State::Done;
                    } else {
                        // data rc'd..
                        // suck into epilog.
                        rc = dev_.read(v);
                        if (pdu.epilogue_len >= sizeof(pdu.epilogue)) { panic0(); }
                        pdu.epilogue[pdu.epilogue_len] = v;
                        pdu.epilogue_len += 1;
                    }
                } break;
                case State::Done:
                    break;
                default:
                    panic0();
            }
        }
    }
    return true;
}

static void write(FlushedIO &dev, CRC16 &crc, SlaveReadMultiple &src)
{
    dev.write(3);
    crc.update(3);
    uint16_t siz = src.len * sizeof(uint16_t);
    dev.write(siz);
    crc.update((uint8_t)siz);
    uint8_t buf[2];
    for (size_t i = 0; i < src.len; ++i) {
        raw::be::encode(src.values[i], buf);
        crc.update(buf, 2);
        dev.write(buf, 2);
    }
}

bool ServerSocket::send(SlavePdu &&pdu)
{
    CRC16 crc;
    dev_.write(addr_);
    crc.update(addr_);
    switch (pdu.opcode) {
        case SlavePdu::ReadRegMulti:
            write(dev_, crc, pdu.reg_multi);
            break;
        default:
            panic0();
    }
    uint8_t buf[2];
    raw::be::encode(crc.value(), buf);
    dev_.write(buf, 2);
    dev_.write(pdu.epilogue, pdu.epilogue_len);
    return true;
}
} // namespace rtu
}; // namespace modbus
