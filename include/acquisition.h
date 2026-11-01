#ifndef ACQUISITION_H
#define ACQUISITION_H
#include <stdint.h>
typedef struct {
    uint32_t sh, icg;
    uint8_t ready;
} AcquisitionClock;
/* Resume only clocks retained after a completed fast one-shot with identical timing. */
uint8_t acquisition_prepare(AcquisitionClock *clock, uint32_t sh, uint32_t icg, uint8_t fast);
void acquisition_finished(AcquisitionClock *clock, uint8_t fast);
#endif
