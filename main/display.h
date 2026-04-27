/* display.h — Public API for the gateway OLED status display.
 *
 * The display module drives a 128×64 SSD1306 panel over I2C.  It owns all
 * hardware initialisation (power rail, reset pulse, I2C probe) and provides
 * one rendering call that the gateway task invokes periodically to keep the
 * panel in sync with the current system state.
 *
 * The panel cycles through up to three pages automatically:
 *   Page 0 – high-level gateway status (mode, LoRa/UART health, last messages)
 *   Page 1 – numeric counters and full downlink/uplink previews
 *   Page 2 – alert page (shown only when a fault condition is active)
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Initialise the SSD1306 OLED panel.
 *
 * Powers the VEXT supply rail, pulses the RST line, probes the I2C bus, and
 * writes a brief splash screen.  Must be called once before any rendering.
 *
 * Returns true on success, false if the display is absent or the I2C probe
 * fails (the gateway continues operating without a display in that case).
 */
bool display_init(void);

/* Render the current gateway state onto the OLED panel.
 *
 * Parameters (all string pointers may be NULL; NULL is treated as "none"):
 *   mode            – current operating mode label (e.g. "READY", "WIFI", "MANUAL")
 *   lora_status     – LoRa link health label (e.g. "ONLINE", "STALE", "IDLE")
 *   cmd_status      – last LoRa uplink command result (e.g. "IDLE", "FAILED")
 *   uart_status     – UART link health label (e.g. "BRIDGE", "QUIET")
 *   last_downlink   – most recent LoRa-received payload (base→robot direction)
 *   last_uplink     – most recent UART-received payload (robot→base direction)
 *   alert_headline  – short alert title shown on the alert page (page 2)
 *   alert_detail    – additional alert context shown on the alert page
 *   lora_rx_count   – running count of LoRa packets received
 *   uart_forward_count – running count of UART frames forwarded to STM32
 *   tx_drop_count   – number of LoRa TX queue drops (0 = healthy)
 *   uart_overflow_count – number of UART RX buffer overflow events (0 = healthy)
 *
 * The function computes which display page is active from the system tick,
 * clears the screen only on a page transition, and skips re-drawing lines
 * whose content has not changed since the last call.
 */
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
