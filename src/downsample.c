#include "downsample.h"
#include "log_encoding.h"
volatile uint8_t pending_encoding = 0, active_encoding = 0;
volatile uint16_t pending_stride = 1, active_stride = 1;
static void be16(uint8_t *p, uint16_t v) { p[0] = v >> 8; p[1] = v; }
uint16_t downsample_frame(volatile uint16_t *samples, uint16_t pixels, uint16_t stride, uint8_t header[16])
{
    uint16_t i, count = 0, minimum = 65535, maximum = 0, clipped = 0, invalid = 0;
    if (!stride || stride > MAX_DOWNSAMPLE || !pixels) return 0;
    for (i = 0; i < pixels; ++i) {
        uint16_t v = samples[i];
        if (v < minimum) minimum = v;
        if (v > maximum) maximum = v;
        if (v == 4095) ++clipped;
        if (v > 4095) ++invalid;
    }
    for (i = 0; i < pixels; i += stride) samples[count++] = samples[i];
    header[0] = 'D'; header[1] = 'S'; header[2] = 'F'; header[3] = '1';
    be16(header + 4, stride); be16(header + 6, count);
    be16(header + 8, minimum); be16(header + 10, maximum);
    be16(header + 12, clipped); be16(header + 14, invalid);
    return count;
}

uint16_t pack_samples(volatile uint16_t *samples, uint16_t count, uint8_t encoding)
{
    uint32_t accumulator = 0;
    uint16_t i, out = 0;
    uint8_t used = 0, bits = encoding == 1 ? 10 : encoding == 2 ? 12 : 8;
    volatile uint8_t *bytes = (volatile uint8_t *)samples;
    if (!encoding) return 2 * count;
    for (i = 0; i < count; ++i) {
        uint16_t value = samples[i];
        /* Invalid ADC values are flagged in full-frame header statistics. */
        if (value > 4095) value = 4095;
        if (encoding == 1) value = log10_lut[value];
        if (encoding == 3) value = log8_lut[value];
        accumulator |= (uint32_t)value << used;
        used += bits;
        while (used >= 8) {
            bytes[out++] = accumulator;
            accumulator >>= 8; used -= 8;
        }
    }
    if (used) bytes[out++] = accumulator;
    return out;
}
