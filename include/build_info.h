#ifndef BUILD_INFO_H
#define BUILD_INFO_H

/* Identity extension v1. The original 10-byte identity remains unchanged. */
#define BUILD_INFO_MAGIC "[g3gg0.v1]"
#define BUILD_INFO_MAX_BYTES 1024

void UART2_SendBuildInfo(void);

#endif
