#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <ctype.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"

#include "ra01s.h"
#include "display.h"

// ---------------- LoRa settings ----------------
#define LORA_FREQ_HZ       915000000
#define LORA_TX_POWER_DBM  22
#define LORA_TCXO_VOLT     3.3f
#define LORA_USE_LDO       true

#define LORA_SF            7
#define LORA_BW            4
#define LORA_CR            1
#define LORA_PREAMBLE      8
#define LORA_PAYLOAD_LEN   0
#define LORA_CRC_ON        true
#define LORA_INVERT_IRQ    false

// ---------------- Gateway Wi-Fi ----------------
#define GW_STA_SSID        "GriffiniPhone"
#define GW_STA_PASS        "12345678"
#define GW_STA_STATIC_IP   "172.20.10.2"
#define GW_STA_GW          "172.20.10.1"
#define GW_STA_NETMASK     "255.255.255.240"
#define GW_WIFI_CFG_PREFIX "GWCFG:WIFI:"
#define GW_WIFI_NVS_NAMESPACE "gw_wifi"
#define GW_WIFI_NVS_KEY_SSID "ssid"
#define GW_WIFI_NVS_KEY_PASS "pass"
#define GW_WIFI_NVS_KEY_IP "ip"
#define GW_WIFI_NVS_KEY_GW "gw"
#define GW_WIFI_NVS_KEY_MASK "mask"

// ---------------- UART settings ----------------
#define GW_UART            UART_NUM_1
#define GW_UART_BAUD       921600
#define GW_UART_TX_GPIO    6
#define GW_UART_RX_GPIO    7

#define UART_RX_BUF_SIZE   16384
#define UART_TX_BUF_SIZE   8192
#define UART_LINE_MAX      1024
#define UART_EVENT_QUEUE_LEN 64
#define LORA_MAX_PAYLOAD   255
#define UART_STREAM_IDLE_FLUSH_MS        20
#define UART_STREAM_SHORT_IDLE_FLUSH_MS  40
#define UART_STREAM_MIN_FLUSH_BYTES      8
#define UART_READ_TIMEOUT_MS             15
#define UART_RX_MAX_BYTES_PER_PASS        128
#define UART_RX_MAX_PASSES_PER_CYCLE      1
#define UART_RX_DRAIN_PAUSE_MS            5
#define UART_RX_LOOP_PAUSE_MS             10
#define UART_BINARY_DROP_MAX_BYTES       3
#define UART_BINARY_HEX_MAX_BYTES        24
#define LORA_STREAM_CHUNK_MAX            220
#define LORA_STREAM_HEADER_RESERVE       24
#define LORA_TX_RETRY_COUNT 5
#define LORA_TX_RETRY_DELAY_MS 25
#define LORA_TX_QUEUE_LEN 128
#define LORA_TX_QUEUE_WAIT_MS 3
#define TASK_STACK_LORA_TX 6144
#define TASK_STACK_LORA_RX 6144
#define TASK_STACK_UART_RX 10240
#define TASK_STACK_DISPLAY 4096
#define TASK_STACK_WIFI_HTTP 6144
#define GATEWAY_APP_CORE 1
#define UART_RX_TASK_CORE tskNO_AFFINITY
#define TASK_PRIO_LORA_TX 4
#define TASK_PRIO_LORA_RX 3
#define TASK_PRIO_UART_RX 1
#define TASK_PRIO_DISPLAY 1
#define LORA_RX_POLL_DELAY_MS           5
#define GATEWAY_ACTIVITY_READY_MS        8000
#define GATEWAY_ACTIVITY_WARN_MS         90000
#define LOW_PRIORITY_UPLINK_MIN_INTERVAL_MS 30
#define MANUAL_DOWNLINK_GUARD_MS         140
#define GATEWAY_MOTION_LOG_THROTTLE_MS   1000

static const char *TAG = "ROBOT_GW";
static bool display_available = false;
static char s_last_downlink[32] = "none";
static char s_last_uplink[32] = "none";
static uint32_t s_lora_rx_count = 0;
static uint32_t s_uart_forward_count = 0;
static char s_gateway_mode[16] = "BOOT";
static char s_lora_status[16] = "IDLE";
static char s_cmd_status[16] = "IDLE";
static char s_uart_status[16] = "IDLE";
static char s_alert_headline[32] = "BRIDGE OK";
static char s_alert_detail[48] = "Waiting for traffic";
static TickType_t s_last_lora_activity_tick = 0;
static TickType_t s_last_uart_activity_tick = 0;
static httpd_handle_t s_http_server = NULL;
static esp_netif_t *s_sta_netif = NULL;
static bool s_wifi_stack_ready = false;
static bool s_wifi_started = false;
static bool s_wifi_connected = false;
static uint32_t s_manual_http_count = 0;
static uint32_t s_last_manual_http_tick = 0;
static char s_last_manual_http_cmd[32] = "none";

typedef struct {
    char ssid[33];
    char pass[65];
    char ip[16];
    char gw[16];
    char netmask[16];
} wifi_runtime_cfg_t;

static wifi_runtime_cfg_t s_wifi_cfg = {
    .ssid = GW_STA_SSID,
    .pass = GW_STA_PASS,
    .ip = GW_STA_STATIC_IP,
    .gw = GW_STA_GW,
    .netmask = GW_STA_NETMASK,
};

typedef struct {
    bool is_stream;
    bool has_end;
    uint32_t seq;
    const char *payload;
} stream_frame_t;

typedef struct {
    char payload[LORA_MAX_PAYLOAD + 1];
} lora_tx_item_t;

static SemaphoreHandle_t lora_tx_lock;
static QueueHandle_t s_lora_tx_queue = NULL;
static QueueHandle_t s_uart_event_queue = NULL;
static uint32_t s_lora_tx_queue_drops = 0;
static uint32_t s_uart_overflow_count = 0;
static uint32_t s_lora_tx_fail_count = 0;
static uint32_t s_noisy_uplink_suppressed = 0;
static TickType_t s_last_noisy_uplink_log_tick = 0;
static TickType_t s_last_low_priority_uplink_tick = 0;
static TickType_t s_last_manual_downlink_tick = 0;
static TickType_t s_last_lora_tx_fail_tick = 0;

static bool rx_seq_init = false;
static uint32_t rx_seq_expected = 0;
static char rx_stream_buf[UART_LINE_MAX];
static int rx_stream_len = 0;

static void lora_send_text_blocking(const char *text);
static void lora_send_text(const char *text);
static uint8_t lora_receive_locked(uint8_t *rx, size_t rx_size);
static char* trim_inplace(char *s);
static void set_status_text(char *target, size_t target_size, const char *value);
static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data);
static esp_err_t start_or_reconfigure_wifi_from_cfg(const wifi_runtime_cfg_t *cfg);

static esp_err_t save_wifi_cfg_to_nvs(const wifi_runtime_cfg_t *cfg) {
    if (!cfg || cfg->ssid[0] == '\0') return ESP_ERR_INVALID_ARG;

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(GW_WIFI_NVS_NAMESPACE, NVS_READWRITE, &nvs);
    if (err != ESP_OK) return err;

    err = nvs_set_str(nvs, GW_WIFI_NVS_KEY_SSID, cfg->ssid);
    if (err == ESP_OK) err = nvs_set_str(nvs, GW_WIFI_NVS_KEY_PASS, cfg->pass);
    if (err == ESP_OK) err = nvs_set_str(nvs, GW_WIFI_NVS_KEY_IP, cfg->ip);
    if (err == ESP_OK) err = nvs_set_str(nvs, GW_WIFI_NVS_KEY_GW, cfg->gw);
    if (err == ESP_OK) err = nvs_set_str(nvs, GW_WIFI_NVS_KEY_MASK, cfg->netmask);
    if (err == ESP_OK) err = nvs_commit(nvs);

    nvs_close(nvs);
    return err;
}

static bool load_wifi_cfg_from_nvs(wifi_runtime_cfg_t *out) {
    if (!out) return false;

    nvs_handle_t nvs = 0;
    esp_err_t err = nvs_open(GW_WIFI_NVS_NAMESPACE, NVS_READONLY, &nvs);
    if (err != ESP_OK) return false;

    wifi_runtime_cfg_t loaded = {0};
    size_t len = sizeof(loaded.ssid);
    err = nvs_get_str(nvs, GW_WIFI_NVS_KEY_SSID, loaded.ssid, &len);
    if (err != ESP_OK || loaded.ssid[0] == '\0') {
        nvs_close(nvs);
        return false;
    }

    len = sizeof(loaded.pass);
    if (nvs_get_str(nvs, GW_WIFI_NVS_KEY_PASS, loaded.pass, &len) != ESP_OK) loaded.pass[0] = '\0';
    len = sizeof(loaded.ip);
    if (nvs_get_str(nvs, GW_WIFI_NVS_KEY_IP, loaded.ip, &len) != ESP_OK) loaded.ip[0] = '\0';
    len = sizeof(loaded.gw);
    if (nvs_get_str(nvs, GW_WIFI_NVS_KEY_GW, loaded.gw, &len) != ESP_OK) loaded.gw[0] = '\0';
    len = sizeof(loaded.netmask);
    if (nvs_get_str(nvs, GW_WIFI_NVS_KEY_MASK, loaded.netmask, &len) != ESP_OK) loaded.netmask[0] = '\0';

    nvs_close(nvs);
    *out = loaded;
    return true;
}

static bool parse_ipv4_text(const char *text, uint8_t *a, uint8_t *b, uint8_t *c, uint8_t *d) {
    if (!text || !a || !b || !c || !d) return false;
    unsigned int o0 = 0, o1 = 0, o2 = 0, o3 = 0;
    if (sscanf(text, "%u.%u.%u.%u", &o0, &o1, &o2, &o3) != 4) return false;
    if (o0 > 255 || o1 > 255 || o2 > 255 || o3 > 255) return false;
    *a = (uint8_t)o0;
    *b = (uint8_t)o1;
    *c = (uint8_t)o2;
    *d = (uint8_t)o3;
    return true;
}

static esp_err_t apply_wifi_sta_config(const wifi_runtime_cfg_t *cfg, bool reconnect_now) {
    if (!cfg || cfg->ssid[0] == '\0') return ESP_ERR_INVALID_ARG;
    if (!s_sta_netif) return ESP_ERR_INVALID_STATE;

    if (cfg->ip[0] != '\0' && cfg->gw[0] != '\0' && cfg->netmask[0] != '\0') {
        esp_netif_ip_info_t ip_info = {0};
        uint8_t a = 0, b = 0, c = 0, d = 0;
        if (!parse_ipv4_text(cfg->ip, &a, &b, &c, &d)) return ESP_ERR_INVALID_ARG;
        esp_netif_set_ip4_addr(&ip_info.ip, a, b, c, d);
        if (!parse_ipv4_text(cfg->gw, &a, &b, &c, &d)) return ESP_ERR_INVALID_ARG;
        esp_netif_set_ip4_addr(&ip_info.gw, a, b, c, d);
        if (!parse_ipv4_text(cfg->netmask, &a, &b, &c, &d)) return ESP_ERR_INVALID_ARG;
        esp_netif_set_ip4_addr(&ip_info.netmask, a, b, c, d);
        esp_netif_dhcpc_stop(s_sta_netif);
        ESP_RETURN_ON_ERROR(esp_netif_set_ip_info(s_sta_netif, &ip_info), TAG, "Failed setting static IP info");
    } else {
        ESP_LOGI(TAG, "Using DHCP for gateway STA (same hotspot as base station)");
        esp_netif_dhcpc_start(s_sta_netif);
    }

    wifi_config_t sta_cfg = {0};
    strncpy((char *)sta_cfg.sta.ssid, cfg->ssid, sizeof(sta_cfg.sta.ssid) - 1);
    sta_cfg.sta.ssid[sizeof(sta_cfg.sta.ssid) - 1] = '\0';
    strncpy((char *)sta_cfg.sta.password, cfg->pass, sizeof(sta_cfg.sta.password) - 1);
    sta_cfg.sta.password[sizeof(sta_cfg.sta.password) - 1] = '\0';
    sta_cfg.sta.scan_method = WIFI_ALL_CHANNEL_SCAN;
    sta_cfg.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
    sta_cfg.sta.threshold.authmode = WIFI_AUTH_OPEN;
    sta_cfg.sta.pmf_cfg.capable = true;
    sta_cfg.sta.pmf_cfg.required = false;
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_STA, &sta_cfg), TAG, "Failed applying STA config");

    if (reconnect_now) {
        esp_wifi_disconnect();
        vTaskDelay(pdMS_TO_TICKS(120));
        ESP_RETURN_ON_ERROR(esp_wifi_connect(), TAG, "Failed reconnecting with new Wi-Fi config");
    }
    return ESP_OK;
}

static bool parse_wifi_cfg_message(const char *payload, wifi_runtime_cfg_t *out) {
    if (!payload || !out) return false;
    if (strncmp(payload, GW_WIFI_CFG_PREFIX, strlen(GW_WIFI_CFG_PREFIX)) != 0) return false;

    char buffer[200];
    snprintf(buffer, sizeof(buffer), "%s", payload + strlen(GW_WIFI_CFG_PREFIX));

    char *ctx = NULL;
    char *ssid = strtok_r(buffer, "|", &ctx);
    char *pass = strtok_r(NULL, "|", &ctx);
    char *ip = strtok_r(NULL, "|", &ctx);
    char *gw = strtok_r(NULL, "|", &ctx);
    char *mask = strtok_r(NULL, "|", &ctx);
    if (!ssid || !pass) return false;

    ssid = trim_inplace(ssid);
    pass = trim_inplace(pass);
    ip = ip ? trim_inplace(ip) : "";
    gw = gw ? trim_inplace(gw) : "";
    mask = mask ? trim_inplace(mask) : "";
    if (ssid[0] == '\0') return false;

    snprintf(out->ssid, sizeof(out->ssid), "%s", ssid);
    snprintf(out->pass, sizeof(out->pass), "%s", pass);
    snprintf(out->ip, sizeof(out->ip), "%s", ip);
    snprintf(out->gw, sizeof(out->gw), "%s", gw);
    snprintf(out->netmask, sizeof(out->netmask), "%s", mask);
    return true;
}

static esp_err_t wifi_stack_init_once(void) {
    if (s_wifi_stack_ready) return ESP_OK;

    esp_err_t nvs_err = nvs_flash_init();
    if (nvs_err == ESP_ERR_NVS_NO_FREE_PAGES || nvs_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_RETURN_ON_ERROR(nvs_flash_erase(), TAG, "nvs erase failed");
        nvs_err = nvs_flash_init();
    }
    ESP_RETURN_ON_ERROR(nvs_err, TAG, "nvs init failed");
    ESP_RETURN_ON_ERROR(esp_netif_init(), TAG, "esp_netif_init failed");
    ESP_RETURN_ON_ERROR(esp_event_loop_create_default(), TAG, "event loop create failed");

    s_sta_netif = esp_netif_create_default_wifi_sta();
    if (!s_sta_netif) return ESP_FAIL;

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&cfg), TAG, "esp_wifi_init failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL, NULL), TAG, "wifi event register failed");
    ESP_RETURN_ON_ERROR(esp_event_handler_instance_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL, NULL), TAG, "ip event register failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_STA), TAG, "set wifi mode failed");

    s_wifi_stack_ready = true;
    return ESP_OK;
}

static esp_err_t start_or_reconfigure_wifi_from_cfg(const wifi_runtime_cfg_t *cfg) {
    ESP_RETURN_ON_ERROR(wifi_stack_init_once(), TAG, "wifi stack init failed");

    if (!s_wifi_started) {
        ESP_RETURN_ON_ERROR(apply_wifi_sta_config(cfg, false), TAG, "apply sta cfg failed");
        ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "wifi start failed");
        s_wifi_started = true;
        set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "WIFI");
        ESP_LOGI(TAG, "Gateway Wi-Fi start requested from LoRa cfg: ssid=%s", cfg->ssid);
        return ESP_OK;
    }

    ESP_RETURN_ON_ERROR(apply_wifi_sta_config(cfg, true), TAG, "reconfigure sta cfg failed");
    set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "WIFI");
    ESP_LOGI(TAG, "Gateway Wi-Fi reconfigured from LoRa cfg: ssid=%s", cfg->ssid);
    return ESP_OK;
}

static bool handle_gateway_control_command(const char *cmd) {
    if (!cmd) return false;
    if (strncmp(cmd, GW_WIFI_CFG_PREFIX, strlen(GW_WIFI_CFG_PREFIX)) != 0) return false;

    // Ignore reflected status/ack frames (e.g. GWCFG:WIFI:OK) to prevent control chatter loops.
    if (strchr(cmd, '|') == NULL) {
        return true;
    }

    wifi_runtime_cfg_t next_cfg = s_wifi_cfg;
    if (!parse_wifi_cfg_message(cmd, &next_cfg)) {
        ESP_LOGW(TAG, "Rejected Wi-Fi cfg command (format): %s", cmd);
        lora_send_text("GWCFG:WIFI:ERR:FORMAT");
        return true;
    }

    esp_err_t err = start_or_reconfigure_wifi_from_cfg(&next_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Rejected Wi-Fi cfg command (apply err=0x%x)", (unsigned)err);
        lora_send_text("GWCFG:WIFI:ERR:APPLY");
        return true;
    }

    err = save_wifi_cfg_to_nvs(&next_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Rejected Wi-Fi cfg persistence (save err=0x%x)", (unsigned)err);
        lora_send_text("GWCFG:WIFI:ERR:SAVE");
        return true;
    }

    s_wifi_cfg = next_cfg;
    set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "WIFI");
    ESP_LOGI(TAG, "Applied Wi-Fi cfg from LoRa: ssid=%s ip=%s gw=%s", s_wifi_cfg.ssid, s_wifi_cfg.ip, s_wifi_cfg.gw);
    lora_send_text_blocking("GWCFG:WIFI:OK");
    ESP_LOGW(TAG, "Restarting gateway to finalize Wi-Fi credential update");
    vTaskDelay(pdMS_TO_TICKS(250));
    esp_restart();
    return true;
}

static char* trim_inplace(char *s) {
    if (!s) return s;

    while (*s == ' ' || *s == '\t' || *s == '\r' || *s == '\n') s++;

    size_t n = strlen(s);
    while (n > 0) {
        char c = s[n - 1];
        if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
            s[n - 1] = '\0';
            n--;
        } else {
            break;
        }
    }

    return s;
}

static void make_printable(const char *in, size_t in_len, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    if (!in || in_len == 0) { out[0] = '\0'; return; }
    size_t j = 0;
    for (size_t i = 0; i < in_len && j + 1 < out_size; i++) {
        unsigned char c = (unsigned char)in[i];
        out[j++] = (c >= 32 && c <= 126) ? (char)c : '.';
    }
    out[j] = '\0';
}

static void update_preview_text(const char *input, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    if (!input) { snprintf(out, out_size, "none"); return; }
    char printable[64];
    make_printable(input, strnlen(input, sizeof(printable) - 1), printable, sizeof(printable));
    char *trimmed = trim_inplace(printable);
    snprintf(out, out_size, "%.*s", (int)((strlen(trimmed) < out_size - 1) ? strlen(trimmed) : out_size - 1), trimmed);
    if (out[0] == '\0') snprintf(out, out_size, "none");
}

static void set_status_text(char *target, size_t target_size, const char *value) {
    if (!target || target_size == 0) return;
    snprintf(target, target_size, "%s", value ? value : "none");
}

static bool matches_any_token(const char *value, const char *const *tokens, size_t token_count) {
    if (!value || !tokens || token_count == 0) return false;
    for (size_t i = 0; i < token_count; i++) {
        if (tokens[i] && strcmp(value, tokens[i]) == 0) {
            return true;
        }
    }
    return false;
}

static bool is_high_priority_manual_uplink(const char *payload) {
    if (!payload || payload[0] == '\0') return false;
    if (strncmp(payload, "ACK:", 4) == 0) return true;
    return strncmp(payload, "F:", 2) == 0 || strncmp(payload, "FAULT", 5) == 0 || strstr(payload, "ESTOP") != NULL;
}

static bool is_manual_downlink_command(const char *payload) {
    static const char *const manual_tokens[] = {
        "MANUAL", "PAUSE", "AUTO",
        "FORWARD", "BACKWARD", "LEFT", "RIGHT", "STOP", "ESTOP",
    };

    if (!payload || payload[0] == '\0') return false;

    // Existing joystick + drive formats
    if (strncmp(payload, "D:", 2) == 0 ||
        strncmp(payload, "J:", 2) == 0 ||
        strncmp(payload, "DRIVE,", 6) == 0) {
        return true;
    }

    if (strncmp(payload, "TEST SALT", 9) == 0) return true;
    if (strncmp(payload, "TEST BRINE", 10) == 0) return true;

    return matches_any_token(payload, manual_tokens,
        sizeof(manual_tokens) / sizeof(manual_tokens[0]));
}

static bool should_suppress_uplink_in_manual_mode(const char *payload) {
    (void)payload;
    return false;
}

static bool is_noisy_console_uplink(const char *payload) {
    if (!payload || payload[0] == '\0') return false;
    return strstr(payload, "[HEALTH] GPS: TIMEOUT") != NULL ||
           strstr(payload, "[HEALTH] GPS: DEGRADED") != NULL ||
           strstr(payload, "[RC] Health check") != NULL ||
           strstr(payload, "[CONSOLE] USART2 RX recovered") != NULL;
}

static bool should_drop_uart_uplink(const char *payload) {
    if (!payload || payload[0] == '\0') return false;
    if (is_high_priority_manual_uplink(payload)) return false;
    if (is_noisy_console_uplink(payload)) return true;
    return should_suppress_uplink_in_manual_mode(payload);
}

static bool should_rate_limit_low_priority_uplink(const char *payload) {
    if (!payload || payload[0] == '\0') return false;
    if (is_high_priority_manual_uplink(payload)) return false;
    TickType_t now = xTaskGetTickCount();
    if (s_last_manual_downlink_tick != 0 && (now - s_last_manual_downlink_tick) < pdMS_TO_TICKS(MANUAL_DOWNLINK_GUARD_MS)) return true;
    if (s_last_low_priority_uplink_tick != 0 && (now - s_last_low_priority_uplink_tick) < pdMS_TO_TICKS(LOW_PRIORITY_UPLINK_MIN_INTERVAL_MS)) return true;
    s_last_low_priority_uplink_tick = now;
    return false;
}

static bool is_gateway_ack_or_status_frame(const char *payload) {
    if (!payload || payload[0] == '\0') return false;
    return strncmp(payload, "GWRX:", 5) == 0 ||
           strncmp(payload, "GWCFG:WIFI:OK", 13) == 0 ||
           strncmp(payload, "GWCFG:WIFI:ERR", 14) == 0;
}

static bool preview_present(const char *value) {
    return value && value[0] != '\0' && strcmp(value, "none") != 0;
}

static void note_lora_activity(void) {
    s_last_lora_activity_tick = xTaskGetTickCount();
    set_status_text(s_lora_status, sizeof(s_lora_status), "ONLINE");
}

static void note_uart_activity(void) {
    s_last_uart_activity_tick = xTaskGetTickCount();
    set_status_text(s_uart_status, sizeof(s_uart_status), "BRIDGE");
}

static void refresh_gateway_display_states(void) {
    const TickType_t now = xTaskGetTickCount();
    const bool lora_seen = (s_lora_rx_count > 0) || preview_present(s_last_downlink);
    const bool uart_seen = (s_uart_forward_count > 0) || preview_present(s_last_uplink);
    const uint32_t lora_age_ms = s_last_lora_activity_tick == 0 ? UINT32_MAX : (uint32_t)((now - s_last_lora_activity_tick) * portTICK_PERIOD_MS);
    const uint32_t uart_age_ms = s_last_uart_activity_tick == 0 ? UINT32_MAX : (uint32_t)((now - s_last_uart_activity_tick) * portTICK_PERIOD_MS);
    if (!lora_seen) set_status_text(s_lora_status, sizeof(s_lora_status), "IDLE");
    else if (lora_age_ms <= GATEWAY_ACTIVITY_READY_MS) set_status_text(s_lora_status, sizeof(s_lora_status), "ONLINE");
    else if (lora_age_ms <= GATEWAY_ACTIVITY_WARN_MS) set_status_text(s_lora_status, sizeof(s_lora_status), "QUIET");
    else set_status_text(s_lora_status, sizeof(s_lora_status), "STALE");
    if (!uart_seen) set_status_text(s_uart_status, sizeof(s_uart_status), "IDLE");
    else if (uart_age_ms <= GATEWAY_ACTIVITY_READY_MS) set_status_text(s_uart_status, sizeof(s_uart_status), "BRIDGE");
    else if (uart_age_ms <= GATEWAY_ACTIVITY_WARN_MS) set_status_text(s_uart_status, sizeof(s_uart_status), "QUIET");
    else set_status_text(s_uart_status, sizeof(s_uart_status), "STALE");
    if (s_wifi_connected) set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "READY");
    else set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "WIFI");
}

static stream_frame_t parse_stream_frame(char *text) {
    stream_frame_t out = { .is_stream = false, .has_end = true, .seq = 0, .payload = text };
    if (!text || strncmp(text, "S:", 2) != 0) return out;
    char *p = text + 2;
    char *endptr = NULL;
    unsigned long seq = strtoul(p, &endptr, 10);
    if (endptr == p || !endptr || *endptr != ':') return out;
    out.is_stream = true;
    out.seq = (uint32_t)seq;
    char *payload = endptr + 1;
    if ((payload[0] == 'M' || payload[0] == 'E') && payload[1] == ':') {
        out.has_end = (payload[0] == 'E');
        out.payload = payload + 2;
    } else {
        out.has_end = true;
        out.payload = payload;
    }
    return out;
}

static void forward_command_to_stm32(const char *cmd, const char *source, bool always_log) {
    if (!cmd || cmd[0] == '\0') return;
    const bool manual_downlink = is_manual_downlink_command(cmd);
    char line[UART_LINE_MAX];
    int w = snprintf(line, sizeof(line), "%s\r\n", cmd);
    if (w <= 0) return;
    uart_write_bytes(GW_UART, line, w);
    if (manual_downlink) {
        s_last_manual_downlink_tick = xTaskGetTickCount();
    }
    note_uart_activity();
    s_uart_forward_count++;
    update_preview_text(cmd, s_last_uplink, sizeof(s_last_uplink));
    if (always_log || manual_downlink) {
        ESP_LOGI(TAG, "%s -> STM32: %s", source ? source : "CMD", cmd);
    }
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    refresh_gateway_display_states();
    char body[512];
    const uint32_t now_ms = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    const uint32_t manual_age_ms = s_last_manual_http_tick == 0 ? 0xFFFFFFFFu : (now_ms - s_last_manual_http_tick);
    snprintf(body, sizeof(body),
             "{\"ok\":true,\"mode\":\"manual-gateway\",\"wifiConnected\":%s,\"manualReady\":%s,\"lastManualCommand\":\"%s\",\"manualCommandCount\":%lu,\"manualCommandAgeMs\":%lu,\"lastLoRaDownlink\":\"%s\",\"lastStmForward\":\"%s\",\"loraRxCount\":%lu,\"uartForwardCount\":%lu,\"gatewayState\":\"%s\",\"loraStatus\":\"%s\",\"uartStatus\":\"%s\"}",
             s_wifi_connected ? "true" : "false",
             s_wifi_connected ? "true" : "false",
             s_last_manual_http_cmd,
             (unsigned long)s_manual_http_count,
             (unsigned long)manual_age_ms,
             s_last_downlink,
             s_last_uplink,
             (unsigned long)s_lora_rx_count,
             (unsigned long)s_uart_forward_count,
             s_gateway_mode, s_lora_status, s_uart_status);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, body);
}

static void extract_command_from_body(char *body, char *out, size_t out_size) {
    if (!out || out_size == 0) return;
    out[0] = '\0';
    if (!body) return;
    char *trimmed = trim_inplace(body);
    if (*trimmed == '{') {
        char *cmd = strstr(trimmed, "\"cmd\"");
        if (cmd) {
            cmd = strchr(cmd, ':');
            if (cmd) {
                cmd++;
                while (*cmd && (*cmd == ' ' || *cmd == '\t' || *cmd == '"')) cmd++;
                size_t j = 0;
                while (cmd[j] && cmd[j] != '"' && cmd[j] != '}' && j + 1 < out_size) {
                    out[j] = cmd[j];
                    j++;
                }
                out[j] = '\0';
            }
        }
    }
    if (out[0] == '\0') snprintf(out, out_size, "%s", trimmed);
    char *clean = trim_inplace(out);
    for (char *p = clean; *p; ++p) *p = (char)toupper((unsigned char)*p);
    if (clean != out) memmove(out, clean, strlen(clean) + 1);
}

static esp_err_t command_post_handler(httpd_req_t *req) {
    char body[192];
    int received = httpd_req_recv(req, body, sizeof(body) - 1);
    if (received <= 0) return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "empty body");
    body[received] = '\0';
    char command[64];
    extract_command_from_body(body, command, sizeof(command));
    if (command[0] == '\0') return httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "missing cmd");
    s_manual_http_count++;
    s_last_manual_http_tick = (uint32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS);
    update_preview_text(command, s_last_manual_http_cmd, sizeof(s_last_manual_http_cmd));
    set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "MANUAL");
    forward_command_to_stm32(command, "HTTP", true);
    char response[160];
    snprintf(response, sizeof(response), "{\"ok\":true,\"forwarded\":true,\"cmd\":\"%s\",\"count\":%lu}", command, (unsigned long)s_manual_http_count);
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, response);
}

static httpd_handle_t start_http_server(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 6;
    config.stack_size = 6144;
    config.server_port = 80;
    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t status_uri = {.uri = "/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = NULL};
        httpd_uri_t command_uri = {.uri = "/command", .method = HTTP_POST, .handler = command_post_handler, .user_ctx = NULL};
        httpd_register_uri_handler(server, &status_uri);
        httpd_register_uri_handler(server, &command_uri);
        ESP_LOGI(TAG, "Gateway HTTP server ready: GET /status, POST /command");
    }
    return server;
}

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    (void)arg;
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        const wifi_event_sta_disconnected_t *disc = (const wifi_event_sta_disconnected_t *)event_data;
        int reason = disc ? (int)disc->reason : -1;
        s_wifi_connected = false;
        set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "WIFI");
        ESP_LOGW(TAG, "Gateway Wi-Fi disconnected (reason=%d), retrying", reason);
        vTaskDelay(pdMS_TO_TICKS(1200));
        esp_wifi_connect();
    } else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        const ip_event_got_ip_t *got_ip = (const ip_event_got_ip_t *)event_data;
        char ip_text[16] = {0};
        s_wifi_connected = true;
        set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "READY");
        if (!s_http_server) s_http_server = start_http_server();
        if (got_ip) {
            snprintf(ip_text, sizeof(ip_text), IPSTR, IP2STR(&got_ip->ip_info.ip));
        } else {
            snprintf(ip_text, sizeof(ip_text), "%s", s_wifi_cfg.ip[0] ? s_wifi_cfg.ip : "unknown");
        }
        ESP_LOGI(TAG, "Gateway Wi-Fi connected and ready for manual control at http://%s", ip_text);
    }
}

static void wifi_setup(void) {
    snprintf(s_wifi_cfg.ssid, sizeof(s_wifi_cfg.ssid), "%s", GW_STA_SSID);
    snprintf(s_wifi_cfg.pass, sizeof(s_wifi_cfg.pass), "%s", GW_STA_PASS);
    snprintf(s_wifi_cfg.ip, sizeof(s_wifi_cfg.ip), "%s", GW_STA_STATIC_IP);
    snprintf(s_wifi_cfg.gw, sizeof(s_wifi_cfg.gw), "%s", GW_STA_GW);
    snprintf(s_wifi_cfg.netmask, sizeof(s_wifi_cfg.netmask), "%s", GW_STA_NETMASK);
    ESP_LOGI(TAG, "Using hardcoded gateway Wi-Fi cfg: ssid=%s", s_wifi_cfg.ssid);

    esp_err_t err = start_or_reconfigure_wifi_from_cfg(&s_wifi_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Gateway Wi-Fi startup failed (err=0x%x); LoRa config updates can still reconfigure it later", (unsigned)err);
    }
}

static void validate_pin_map(void) {
    const int lora_pins[] = {5, 8, 9, 10, 11, 12, 13, 14};
    for (size_t i = 0; i < (sizeof(lora_pins) / sizeof(lora_pins[0])); i++) {
        if (GW_UART_TX_GPIO == lora_pins[i] || GW_UART_RX_GPIO == lora_pins[i]) {
            ESP_LOGW(TAG, "Potential pin conflict: UART pin overlaps LoRa SPI/control GPIO %d", lora_pins[i]);
        }
    }
}

static void lora_setup(void) {
    ESP_LOGI(TAG, "LoRaInit...");
    LoRaInit();
    int rc = LoRaBegin(LORA_FREQ_HZ, LORA_TX_POWER_DBM, LORA_TCXO_VOLT, LORA_USE_LDO);
    if (rc != 0) {
        ESP_LOGE(TAG, "LoRaBegin failed (%d). Check SX1262 pins/TCXO/freq.", rc);
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    LoRaConfig(LORA_SF, LORA_BW, LORA_CR, LORA_PREAMBLE, LORA_PAYLOAD_LEN, LORA_CRC_ON, LORA_INVERT_IRQ);
    ESP_LOGI(TAG, "LoRa ready @ %d Hz", (int)LORA_FREQ_HZ);
}

static void uart_setup(void) {
    validate_pin_map();
    uart_config_t cfg = {
        .baud_rate = GW_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT
    };
    ESP_ERROR_CHECK(uart_driver_install(GW_UART, UART_RX_BUF_SIZE, UART_TX_BUF_SIZE, UART_EVENT_QUEUE_LEN, &s_uart_event_queue, 0));
    ESP_ERROR_CHECK(uart_param_config(GW_UART, &cfg));
    ESP_ERROR_CHECK(uart_set_pin(GW_UART, GW_UART_TX_GPIO, GW_UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_set_rx_full_threshold(GW_UART, 128));
    ESP_ERROR_CHECK(uart_set_rx_timeout(GW_UART, 20));
    uart_set_always_rx_timeout(GW_UART, true);
    ESP_ERROR_CHECK(gpio_pullup_en(GW_UART_RX_GPIO));
    ESP_ERROR_CHECK(gpio_pulldown_dis(GW_UART_RX_GPIO));
    set_status_text(s_uart_status, sizeof(s_uart_status), "READY");
    ESP_LOGI(TAG, "UART ready: UART%d TX=%d RX=%d @ %d", (int)GW_UART, GW_UART_TX_GPIO, GW_UART_RX_GPIO, GW_UART_BAUD);
}

static void lora_send_text_blocking(const char *text) {
    if (!text) return;
    size_t n = strnlen(text, UART_LINE_MAX - 1);
    if (n == 0) return;
    if (n > LORA_MAX_PAYLOAD) {
        ESP_LOGW(TAG, "LoRa payload too long (%u), truncating to %u", (unsigned)n, (unsigned)LORA_MAX_PAYLOAD);
        n = LORA_MAX_PAYLOAD;
    }
    xSemaphoreTake(lora_tx_lock, portMAX_DELAY);
    bool sent = false;
    for (int retry = 0; retry < LORA_TX_RETRY_COUNT; retry++) {
        bool send_result = LoRaSend((uint8_t*)text, (uint8_t)n, SX126x_TXMODE_SYNC);
        if (send_result) { sent = true; break; }
        ESP_LOGW(TAG, "LoRa uplink retry %d/%d failed", retry + 1, LORA_TX_RETRY_COUNT);
        vTaskDelay(pdMS_TO_TICKS(LORA_TX_RETRY_DELAY_MS));
    }
    xSemaphoreGive(lora_tx_lock);
    if (!sent) {
        s_lora_tx_fail_count++;
        s_last_lora_tx_fail_tick = xTaskGetTickCount();
        set_status_text(s_cmd_status, sizeof(s_cmd_status), "FAILED");
        char preview[48];
        make_printable(text, n, preview, sizeof(preview));
        ESP_LOGE(TAG, "LoRa uplink failed after %d attempts (%u bytes): %s", LORA_TX_RETRY_COUNT, (unsigned)n, preview);
    }
}

static uint8_t lora_receive_locked(uint8_t *rx, size_t rx_size) {
    if (!rx || rx_size == 0) return 0;
    xSemaphoreTake(lora_tx_lock, portMAX_DELAY);
    uint8_t count = LoRaReceive(rx, (int16_t)(rx_size - 1));
    xSemaphoreGive(lora_tx_lock);
    return count;
}

static void lora_send_text(const char *text) {
    if (!text) return;
    if (!s_lora_tx_queue) { lora_send_text_blocking(text); return; }
    lora_tx_item_t item = {0};
    snprintf(item.payload, sizeof(item.payload), "%s", text);
    if (xQueueSend(s_lora_tx_queue, &item, pdMS_TO_TICKS(LORA_TX_QUEUE_WAIT_MS)) != pdTRUE) {
        s_lora_tx_queue_drops++;
        if ((s_lora_tx_queue_drops % 25u) == 1u) ESP_LOGW(TAG, "LoRa TX queue full, dropping payload (drops=%lu)", (unsigned long)s_lora_tx_queue_drops);
    }
}

static void lora_tx_task(void *arg) {
    (void)arg;
    lora_tx_item_t item;
    for (;;) {
        if (xQueueReceive(s_lora_tx_queue, &item, portMAX_DELAY) == pdTRUE) {
            lora_send_text_blocking(item.payload);
        }
    }
}

static void lora_rx_task(void *arg) {
    (void)arg;
    uint8_t rx[255];
    TickType_t last_motion_log_tick = 0;
    const TickType_t manual_log_interval_ticks = pdMS_TO_TICKS(GATEWAY_MOTION_LOG_THROTTLE_MS);
    while (1) {
        uint8_t n = lora_receive_locked(rx, sizeof(rx));
        if (n > 0) {
            if (n >= sizeof(rx)) n = sizeof(rx) - 1;
            rx[n] = '\0';
            char rx_copy[UART_LINE_MAX];
            strncpy(rx_copy, (char*)rx, sizeof(rx_copy) - 1);
            rx_copy[sizeof(rx_copy) - 1] = '\0';
            char *cmd = trim_inplace(rx_copy);
            update_preview_text(cmd, s_last_downlink, sizeof(s_last_downlink));
            s_lora_rx_count++;
            note_lora_activity();
            if (is_gateway_ack_or_status_frame(cmd)) {
                vTaskDelay(pdMS_TO_TICKS(LORA_RX_POLL_DELAY_MS));
                continue;
            }
            bool motion = is_manual_downlink_command(cmd);
            TickType_t now = xTaskGetTickCount();
            bool should_log_motion = !motion || last_motion_log_tick == 0 || (now - last_motion_log_tick) >= manual_log_interval_ticks;
            if (should_log_motion && motion) last_motion_log_tick = now;
            if (handle_gateway_control_command(cmd)) {
                vTaskDelay(pdMS_TO_TICKS(LORA_RX_POLL_DELAY_MS));
                continue;
            }
            stream_frame_t frame = parse_stream_frame(cmd);
            const char *uart_payload = frame.payload ? frame.payload : "";
            if (frame.is_stream) {
                if (!rx_seq_init) { rx_seq_expected = frame.seq; rx_seq_init = true; }
                else if (frame.seq != rx_seq_expected) { rx_stream_len = 0; rx_seq_expected = frame.seq; }
                size_t payload_len = strlen(uart_payload);
                if (payload_len > 0 && uart_payload[payload_len - 1] == '\n') payload_len--;
                int remain = (int)sizeof(rx_stream_buf) - 1 - rx_stream_len;
                if (remain > 0 && payload_len > 0) {
                    size_t copy_len = payload_len > (size_t)remain ? (size_t)remain : payload_len;
                    memcpy(&rx_stream_buf[rx_stream_len], uart_payload, copy_len);
                    rx_stream_len += (int)copy_len;
                    rx_stream_buf[rx_stream_len] = '\0';
                }
                rx_seq_expected = frame.seq + 1;
                if (!frame.has_end) { vTaskDelay(pdMS_TO_TICKS(LORA_RX_POLL_DELAY_MS)); continue; }
                forward_command_to_stm32(rx_stream_buf, "LoRa", should_log_motion);
                rx_stream_len = 0;
                rx_stream_buf[0] = '\0';
            } else {
                forward_command_to_stm32(uart_payload, "LoRa", should_log_motion);
            }
            char ack[80];
            snprintf(ack, sizeof(ack), "GWRX:%.69s", cmd);
            lora_send_text(ack);
        }
        vTaskDelay(pdMS_TO_TICKS(LORA_RX_POLL_DELAY_MS));
    }
}

static void gateway_display_task(void *arg) {
    (void)arg;
    TickType_t next_display_init_retry = 0;
    while (1) {
        if (!display_available) {
            TickType_t now = xTaskGetTickCount();
            if (now >= next_display_init_retry) {
                ESP_LOGW(TAG, "Display unavailable, retrying OLED init");
                display_available = display_init();
                next_display_init_retry = now + pdMS_TO_TICKS(5000);
            }
        }
        if (display_available) {
            refresh_gateway_display_states();
            display_show_gateway_status(s_gateway_mode,
                                        s_lora_status,
                                        s_cmd_status,
                                        s_uart_status,
                                        s_last_downlink,
                                        s_last_uplink,
                                        s_alert_headline,
                                        s_alert_detail,
                                        s_lora_rx_count,
                                        s_uart_forward_count,
                                        s_lora_tx_queue_drops,
                                        s_uart_overflow_count);
        }
        vTaskDelay(pdMS_TO_TICKS(700));
    }
}

static void uart_rx_task(void *arg) {
    (void)arg;
    char line[UART_LINE_MAX];
    int line_len = 0;
    bool line_chunked = false;
    uint32_t stream_seq = 0;
    const int stream_payload_max = (LORA_MAX_PAYLOAD > LORA_STREAM_HEADER_RESERVE) ? (LORA_MAX_PAYLOAD - LORA_STREAM_HEADER_RESERVE) : LORA_MAX_PAYLOAD;
    const int chunk_max = (stream_payload_max < LORA_STREAM_CHUNK_MAX) ? stream_payload_max : LORA_STREAM_CHUNK_MAX;
    TickType_t last_byte_tick = 0;
    for (;;) {
        bool saw_uart_data = false;
        if (s_uart_event_queue) {
            uart_event_t event;
            if (xQueueReceive(s_uart_event_queue, &event, pdMS_TO_TICKS(UART_READ_TIMEOUT_MS)) == pdTRUE) {
                switch (event.type) {
                    case UART_DATA: saw_uart_data = true; break;
                    case UART_FIFO_OVF:
                    case UART_BUFFER_FULL:
                        s_uart_overflow_count++;
                        ESP_LOGW(TAG, "UART overflow (type=%d count=%lu), flushing input", (int)event.type, (unsigned long)s_uart_overflow_count);
                        uart_flush_input(GW_UART);
                        xQueueReset(s_uart_event_queue);
                        break;
                    case UART_PARITY_ERR:
                    case UART_FRAME_ERR:
                        ESP_LOGW(TAG, "UART line error type=%d", (int)event.type);
                        break;
                    default: break;
                }
            }
        } else {
            vTaskDelay(pdMS_TO_TICKS(UART_READ_TIMEOUT_MS));
        }
        if (saw_uart_data) {
            for (int pass = 0; pass < UART_RX_MAX_PASSES_PER_CYCLE; ++pass) {
                int read_count = uart_read_bytes(GW_UART, (uint8_t *)line + 0, UART_RX_MAX_BYTES_PER_PASS, pdMS_TO_TICKS(0));
                if (read_count <= 0) break;
                note_uart_activity();
                for (int i = 0; i < read_count; i++) {
                    char c = line[i];
                    if ((unsigned char)c < 0x09 || (unsigned char)c == 0x7F) continue;
                    if (c == '\r') continue;
                    if (c == '\n') {
                        if (line_len == 0) continue;
                        line[line_len] = '\0';
                        char *trimmed = trim_inplace(line);
                        if (trimmed[0] == '\0') { line_len = 0; line_chunked = false; last_byte_tick = 0; continue; }
                        update_preview_text(trimmed, s_last_uplink, sizeof(s_last_uplink));
                        if (!should_drop_uart_uplink(trimmed) && !should_rate_limit_low_priority_uplink(trimmed)) {
                            if ((int)strlen(trimmed) > chunk_max) {
                                size_t total = strlen(trimmed);
                                size_t offset = 0;
                                while (offset < total) {
                                    size_t remaining = total - offset;
                                    size_t part = remaining > (size_t)chunk_max ? (size_t)chunk_max : remaining;
                                    char framed[LORA_MAX_PAYLOAD + 1];
                                    const char marker = (offset + part < total) ? 'M' : 'E';
                                    snprintf(framed, sizeof(framed), "S:%lu:%c:%.*s", (unsigned long)stream_seq++, marker, (int)part, trimmed + offset);
                                    lora_send_text(framed);
                                    offset += part;
                                }
                            } else {
                                lora_send_text(trimmed);
                            }
                        }
                        line_len = 0;
                        line_chunked = false;
                        last_byte_tick = 0;
                    } else {
                        if (line_len < UART_LINE_MAX - 1) { line[line_len++] = c; last_byte_tick = xTaskGetTickCount(); }
                        else { line_chunked = true; }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(UART_RX_DRAIN_PAUSE_MS));
            }
        }
        if (line_len > 0 && last_byte_tick != 0) {
            TickType_t now = xTaskGetTickCount();
            const TickType_t idle_flush_ticks = line_chunked ? pdMS_TO_TICKS(UART_STREAM_SHORT_IDLE_FLUSH_MS) : pdMS_TO_TICKS(UART_STREAM_IDLE_FLUSH_MS);
            if ((now - last_byte_tick) >= idle_flush_ticks) {
                line[line_len] = '\0';
                char *trimmed = trim_inplace(line);
                if (trimmed[0] != '\0' && !should_drop_uart_uplink(trimmed) && !should_rate_limit_low_priority_uplink(trimmed)) {
                    lora_send_text(trimmed);
                }
                line_len = 0;
                line_chunked = false;
                last_byte_tick = 0;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(UART_RX_LOOP_PAUSE_MS));
    }
}

void app_main(void) {
    lora_tx_lock = xSemaphoreCreateMutex();
    s_lora_tx_queue = xQueueCreate(LORA_TX_QUEUE_LEN, sizeof(lora_tx_item_t));
    if (!lora_tx_lock || !s_lora_tx_queue) {
        ESP_LOGE(TAG, "Failed to create gateway sync primitives");
        while (1) vTaskDelay(pdMS_TO_TICKS(1000));
    }
    uart_setup();
    lora_setup();
    wifi_setup();
    display_available = display_init();
    xTaskCreatePinnedToCore(lora_tx_task, "lora_tx", TASK_STACK_LORA_TX, NULL, TASK_PRIO_LORA_TX, NULL, GATEWAY_APP_CORE);
    xTaskCreatePinnedToCore(lora_rx_task, "lora_rx", TASK_STACK_LORA_RX, NULL, TASK_PRIO_LORA_RX, NULL, GATEWAY_APP_CORE);
    xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", TASK_STACK_UART_RX, NULL, TASK_PRIO_UART_RX, NULL, UART_RX_TASK_CORE);
    if (display_available) {
        xTaskCreatePinnedToCore(gateway_display_task, "gw_display", TASK_STACK_DISPLAY, NULL, TASK_PRIO_DISPLAY, NULL, GATEWAY_APP_CORE);
    }
    set_status_text(s_gateway_mode, sizeof(s_gateway_mode), "LORA");
    ESP_LOGI(TAG, "Robot gateway running: LoRa bridge active, Wi-Fi startup uses saved or built-in config");
}
