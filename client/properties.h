//#define WIFI_SSID "TP-Link_B24D"
//#define WIFI_PASS "87258400"
#define WIFI_SSID "FrechLan"
#define WIFI_PASS "ctx#3942!"
#define MQTT_BROKER "192.168.0.164"

// Configure the pins used for the ESP32 connection
#if defined(ADAFRUIT_FEATHER_M4_EXPRESS) || \
  defined(ADAFRUIT_FEATHER_M0) || \
  defined(ADAFRUIT_FEATHER_M0_EXPRESS) || \
  defined(ARDUINO_AVR_FEATHER32U4) || \
  defined(ARDUINO_NRF52840_FEATHER) || \
  defined(ADAFRUIT_ITSYBITSY_M0) || \
  defined(ADAFRUIT_ITSYBITSY_M4_EXPRESS) || \
  defined(ARDUINO_AVR_ITSYBITSY32U4_3V) || \
  defined(ARDUINO_NRF52_ITSYBITSY)
  // Configure the pins used for the ESP32 connection
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    2   // Chip select pin
  #define ESP32_RESETN  4   // Reset pin
  #define SPIWIFI_ACK   3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(ARDUINO_AVR_FEATHER328P)
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS     2   // Chip select pin
  #define ESP32_RESETN   4   // Reset pin
  #define SPIWIFI_ACK    3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(TEENSYDUINO)
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS     2   // Chip select pin
  #define ESP32_RESETN   4   // Reset pin
  #define SPIWIFI_ACK    3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif defined(ARDUINO_NRF52832_FEATHER )
  #define SPIWIFI       SPI  // The SPI port
  #define SPIWIFI_SS    2   // Chip select pin
  #define ESP32_RESETN  4   // Reset pin
  #define SPIWIFI_ACK    3   // a.k.a BUSY or READY pin
  #define ESP32_GPIO0   -1
#elif !defined(SPIWIFI_SS)   // if the wifi definition isnt in the board variant
  // Don't change the names of these #define's! they match the variant ones
  #define SPIWIFI       SPI
  #define SPIWIFI_SS    2   // Chip select pin
  #define SPIWIFI_ACK    3   // a.k.a BUSY or READY pin
  #define ESP32_RESETN   4   // Reset pin
  #define ESP32_GPIO0   -1   // Not connected
#endif
