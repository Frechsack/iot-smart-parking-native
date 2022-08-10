#include <MQTT.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "properties.h"

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

int wifi_status = WL_IDLE_STATUS;

enum DeviceTypeName{
  LAMP, PARKING_GUIDE_LAMP, CWO_SENSOR, SERVO
  };

WiFiClient wifi_client;
MQTTClient mqtt_client;

void connect() {
  Serial.print("Check WiFi status: ");
  while (WiFi.status() != WL_CONNECTED) 
    delay(1000);
  Serial.println("done");

  Serial.print("Connect Mqtt-Broker: ");
  while (!mqtt_client.connect("",false))
    delay(1000);
  Serial.println("done");

  mqtt_client.subscribe("instruction",1);
  mqtt_client.subscribe("scan",1);
}

String device_type_name_to_string (const DeviceTypeName jshs) {
  if (jshs == LAMP) {
    return "LAMP";
  }

  else if (jshs == PARKING_GUIDE_LAMP){
    return "PARKING_GUIDE_LAMP";
    }
  else if (jshs == CWO_SENSOR){
    return "CWO_SENSOR";
    }
  else{
    return "SERVO"; 
    }
  };


void scan(){};
void instruction(const String &mac, const String &instruction){};
void registering(const String &mac, const DeviceTypeName &device_type, const int &parking_lot_number, const String &parent_mac){
  String parking_lot_number_as_string = parking_lot_number == -1? "": parking_lot_number+"";
  
  String message = mac + ":" + device_type_name_to_string(device_type) + ": " + parking_lot_number_as_string + ": " + parent_mac;
  mqtt_client.publish("register", message);
  }

void message_received(String &topic, String &payload) {
  if (topic == "scan") {
    scan();
    Serial.println("received something on scan-lane");
  }

  else {
    int payload_finder = payload.indexOf(":");
    String mac = payload.substring(0, (payload_finder -1));
    String instruction = payload.substring(payload_finder+1);
    Serial.println("received on instruction-lane for mac: " + mac + " the instruction: " + instruction);
    }
  
  
}


void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  // Registriere WiFi-Modul
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  while (WiFi.status() == WL_NO_MODULE) {
    Serial.println("WiFi-Connection failed.");
    delay(1000);
  }

  Serial.print("Connect to WiFI: ");
  Serial.print(WIFI_SSID);
  Serial.print(": ");
  do {
    wifi_status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(100);
  } while (wifi_status != WL_CONNECTED);
  Serial.println("done");
  
  mqtt_client.begin(MQTT_BROKER, wifi_client);
  mqtt_client.onMessage(message_received);

  connect();
}

void loop() {
  if (!mqtt_client.connected()) 
    connect();

  mqtt_client.loop();
}
