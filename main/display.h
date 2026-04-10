#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool display_init(void);
void display_show_gateway_status(const char *mode,
                                 const char *lora_status,
                                 const char *uart_status,
                                 const char *last_downlink,
                                 const char *last_uplink,
                                 uint32_t lora_rx_count,
                                 uint32_t uart_forward_count);

#ifdef __cplusplus
}
#endif
