# Gateway Firmware

ESP-IDF firmware for the robot-side gateway.

## Role

- bridges LoRa traffic from the base station to the STM32 over UART
- exposes a local HTTP endpoint for manual command injection and status
- captures live robot telemetry from UART and republishes it over LoRa/status APIs
- manages the robot-side Wi-Fi configuration used for local gateway access

## Main Files

- `main/robot_gateway.c` - LoRa/UART bridge, local HTTP server, telemetry parsing, Wi-Fi handling
- `components` - radio, display, and supporting ESP-IDF components

## Build

From an ESP-IDF shell:

```bash
idf.py build
idf.py flash
idf.py monitor
```

## Notes

- The project name in `CMakeLists.txt` is still `lora_rx_print`, but the actual firmware behavior is the gateway bridge implemented in `main/robot_gateway.c`.
- Gateway status now includes a local robot telemetry snapshot so browser/operator tools can inspect heading, GPS, and proximity data even before it reaches the full backend stack.
