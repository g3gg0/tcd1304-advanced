#include "main.h"
#include "build_info.h"

#ifdef HAVE_GENERATED_BUILD_INFO
#include "build_info.generated.h"
#else
/* Direct/non-PlatformIO builds still report compiler time; Git is explicitly unknown. */
#define BUILD_INFO_JSON "{\"firmware\":\"tcd1304-advanced\",\"protocol\":1," \
    "\"version\":\"1.1.0\",\"built_at\":\"" __DATE__ " " __TIME__ "\"," \
    "\"build_timezone\":\"unspecified\",\"commit\":null,\"dirty\":null," \
    "\"board\":\"unknown\",\"mcu\":\"STM32F401RE\",\"compiler\":\"" __VERSION__ "\"}"
#endif

static const uint8_t payload[] = BUILD_INFO_JSON;
typedef char build_info_size_limit[(sizeof(payload) - 1 <= BUILD_INFO_MAX_BYTES) ? 1 : -1];

void UART2_SendBuildInfo(void)
{
    const uint16_t length = sizeof(payload) - 1; /* Exclude the C string terminator. */
    uint8_t header[12] = BUILD_INFO_MAGIC;
    header[10] = (uint8_t)(length >> 8);
    header[11] = (uint8_t)length;

    UART2_SendData(header, sizeof(header));
    UART2_SendData(payload, length);
}
