#if !defined(RAW_BE_H)
#    define RAW_BE_H

#    include <inttypes.h> //uint8_t, uint16_t

namespace raw
{
namespace be
{
uint16_t decode(uint8_t const arr[2])
{
    return (arr[0] << 8) | arr[1];
}
void encode(uint16_t v, uint8_t arr[2])
{
    arr[0] = (v >> 8) & 0xff;
    arr[1] = v & 0xff;
}
} // namespace be
} // namespace raw

#endif // defined(RAW_BE_H)
