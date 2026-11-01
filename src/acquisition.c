#include "acquisition.h"
uint8_t acquisition_prepare(AcquisitionClock *clock, uint32_t sh, uint32_t icg, uint8_t fast)
{
    uint8_t resume = fast && clock->ready && clock->sh == sh && clock->icg == icg;
    clock->ready = 0;
    clock->sh = sh;
    clock->icg = icg;
    return resume;
}
void acquisition_finished(AcquisitionClock *clock, uint8_t fast)
{
    clock->ready = fast;
}
