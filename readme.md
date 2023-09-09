# Uncanny eyes for ESP32 and 240x240 tft display.

## Dependencies:
- TFT_eSPI

## Pinout
| ESP32 | Display |
|-------|---------|
|  18   | SCL     |
|  23   | SDA     |
|   4   | RST     |
|   2   | DC      |
|  25   | BL      |
|   5   | CS 1    |
|   21  | CS 2    |


## TFT_eSPI configurations:

### User_Setup_Select.h
```C
#include <User_Setups/Setup24_ST7789.h>            // Setup file for DSTIKE/ESP32/ESP8266 configured for ST7789 240 x 240
```

### Setup24_ST7789.h
```C
// Generic ESP32 setup
#define TFT_MISO 19
#define TFT_MOSI 23
#define TFT_SCLK 18
//#define TFT_CS    -1 // Not connected
#define TFT_DC    2
#define TFT_RST   4  // Connect reset to ensure display initialises

// For NodeMCU - use pin numbers in the form PIN_Dx where Dx is the NodeMCU pin designation
//#define TFT_CS   -1      // Define as not used
//#define TFT_DC   PIN_D1  // Data Command control pin
//#define TFT_RST  PIN_D4  // TFT reset pin (could connect to NodeMCU RST, see next line)
//#define TFT_RST  -1    // TFT reset pin connect to NodeMCU RST, must also then add 10K pull down to TFT SCK
```

