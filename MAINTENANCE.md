# Maintenance and board support

This repository is independently maintained for the DOIT AI Eye Vision
ESP32-S3 board with two 240x240 GC9A01 displays.

The original community project at
`SmartArduino/RoPet_ESPS3_AI_EYE` is no longer actively maintained. This tree
keeps the DOIT-specific board integration and working display path separate
while retaining the original licensing and upstream attribution.

## DOIT AI Eye Vision

- Select `CONFIG_BOARD_TYPE_DOIT_AI_EYE_VISION`; it is the ESP32-S3 default in
  `sdkconfig.defaults.esp32s3`.
- Both GC9A01 panels require the vendor-specific initialization sequence in
  `components/espressif__esp_lcd_gc9a01/esp_lcd_gc9a01.c`. Replacing it with
  the generic GC9A01 sequence causes a black screen on this board.
- The two panels use separate SPI buses at 30 MHz.
- Panels remain disabled until the first complete eye frame has been written
  to GRAM.
- Idle animation refreshes only the iris area instead of repeatedly sending
  complete frames. This avoids the horizontal tearing visible with full-frame
  updates.

## Build

Use ESP-IDF 5.4.1 or newer:

```powershell
idf.py set-target esp32s3
idf.py build
```

The generated `sdkconfig` remains ignored. The board choice and required
ESP32-S3 defaults are tracked in `sdkconfig.defaults.esp32s3`.
