#if !defined(RAW_LE_H)
#    define RAW_LE_H

#    include <inttypes.h> //uint8_t, uint16_t

namespace raw
{
namespace le
{
uint16_t decode(uint8_t arr[2])
{
    return (arr[1] << 8) | arr[0];
}
void encode(uint16_t v, uint8_t arr[2])
{
    arr[1] = (v >> 8) & 0xff;
    arr[0] = v & 0xff;
}
} // namespace le
} // namespace raw

#endif // defined(RAW_LE_H)
