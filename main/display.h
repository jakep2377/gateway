#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

bool display_init(void);
void display_show_gateway_status(const char *mode,
                                 const char *lora_status,
                                 const char *cmd_status,
                                 const char *uart_status,
                                 const char *last_downlink,
                                 const char *last_uplink,
                                 const char *alert_headline,
                                 const char *alert_detail,
                                 uint32_t lora_rx_count,
                                 uint32_t uart_forward_count,
                                 uint32_t tx_drop_count,
                                 uint32_t uart_overflow_count);

#ifdef __cplusplus
}
#endif
