#ifndef DOWNSAMPLE_H
#define DOWNSAMPLE_H
#include <stdint.h>
#define MAX_DOWNSAMPLE 64
extern volatile uint16_t pending_stride, active_stride;
extern volatile uint8_t pending_encoding, active_encoding;
/* Full-frame health first, then safe forward in-place decimation. */
uint16_t downsample_frame(volatile uint16_t *samples, uint16_t pixels, uint16_t stride, uint8_t header[16]);
/* Encoding 0=raw16, 1=log10, 2=packed12, 3=log8. LSB-first bit stream. */
uint16_t pack_samples(volatile uint16_t *samples, uint16_t count, uint8_t encoding);
#endif
