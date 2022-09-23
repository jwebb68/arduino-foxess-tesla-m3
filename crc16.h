// -*- mode: c++; indent-tabs-mode: nil; tab-width: 2 -*-
#if !defined(CRC16_H)
#    define CRC16_H

#    include <stddef.h> // size_t
#    include <inttypes.h> // uint8_t

#    if defined(CRC16_8BIT)
class CRC16
{
    public:
        CRC16(): crc_(0xffff) {}
        CRC16(uint16_t initial): crc_(initial) {}

        void update(uint8_t v)
        {
            crc_ = update_(crc_, v);
        }

        void update(uint8_t const *const b, uint8_t const *const e)
        {
            crc_ = update_(crc_, b, e);
        }

        void update(uint8_t const arr[], size_t len)
        {
            update(arr, arr + len);
        }

    private:
        struct UInt16 {
                uint8_t h_;
                uint8_t l_;
                UInt16(uint16_t v): h_((v >> 8) & 0xff), l_(v & 0xff) {}
        };
        UInt16 crc_;
        UInt16 update_(UInt16 crc, uint8_t v);
        UInt16 update_(UInt16 crc, uint8_t const *const b, uint8_t const *const e);
};
#    else // CRC16_16BIT
class CRC16
{
    private:
        uint16_t crc_;
        uint16_t seed_;

    public:
        CRC16(void): CRC16(0xffff) {}

        CRC16(uint16_t seed): crc_(seed), seed_(seed) {}

        void reset(void)
        {
            crc_ = seed_;
        }
        void update(uint8_t v)
        {
            crc_ = update_(crc_, v);
        }

        void update(uint8_t const *const b, uint8_t const *const e)
        {
            crc_ = update_(crc_, b, e);
        }

        void update(uint8_t const arr[], size_t len)
        {
            update(arr, arr + len);
        }

        uint16_t value(void)
        {
            // Note, this number has low and high bytes swapped, so use it accordingly (or swap
            // bytes)
            return crc_;
        }

    private:
        uint16_t update_(uint16_t crc, uint8_t const *const b, uint8_t const *const e);
        uint16_t update_(uint16_t crc, uint8_t v);
};
#    endif

#endif // defined(CRC16_H)
