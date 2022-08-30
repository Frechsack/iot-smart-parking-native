#include <MQTT.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "properties.h"
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

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

enum DeviceTypeName {
  LAMP, PARKING_GUIDE_LAMP, CWO_SENSOR, SERVO
};

enum PhyisicalDeviceTypeName {
  PHYSICAL_SERVO, ADAFRUIT_NEOPIXEL
};

struct DeviceConfiguration {
  String mac;
  DeviceTypeName type;
  PhyisicalDeviceTypeName physical_type;
  int parking_lot_nr;
  String parent_mac;
  boolean is_input;
  boolean is_analog;
  int pin;
  String latest_status;
  int neopixel_position;
};

 Servo servo;

const String NULL_STRING = "";
const int NULL_INT = -1;

const int DEVICES_LENGTH = 1;
DeviceConfiguration DEVICES[DEVICES_LENGTH] = {
  {"SV1", SERVO, PHYSICAL_SERVO, NULL_INT, NULL_STRING, true, true, 46, NULL_STRING, NULL_INT},
  {"PG1", PARKING_GUIDE_LAMP, ADAFRUIT_NEOPIXEL, NULL_INT, NULL_STRING, true, true, 22, NULL_STRING, 1},
};


int wifi_status = WL_IDLE_STATUS;
WiFiClient wifi_client;
MQTTClient mqtt_client;

/**
 * Konvertiert ein enum des Typs "DeviceTypeName" in einen String.
 * @param jshs Der zu konvertierende Typ.
 * @returns Gibt den entsprechenden String zurück.
 */
String device_type_name_to_string (const DeviceTypeName jshs) {
  if (jshs == LAMP) 
    return "LAMP";
  if (jshs == PARKING_GUIDE_LAMP)
    return "PARKING_GUIDE_LAMP";
  if (jshs == CWO_SENSOR)
    return "CWO_SENSOR";
  else
    return "SERVO"; 
};

/**
 * Es wurde eine "scan"-Anweisung per MQTT erhalten.
 */
void reveive_scan(){
  // Verschicke zuerst nur jene ohne parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac == NULL_STRING)
      send_register(DEVICES[i].mac,DEVICES[i].type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac); 

  delay(2000);

    // Dann jene mit parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac != NULL_STRING)
      send_register(DEVICES[i].mac,DEVICES[i].type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac);   
  
};

/**
 * Es wurde eine "instruction"-Anweisung per MQTT erhalten.
 * @param mac Das Gerät, welches geschalten werden soll.
 * @param instruction Der neue anzunehmende Zustand.
 */
void receive_instruction(const String &mac, const String &instruction){
  // Prüfe ob Gerät auf diesem Board
   for(int i = 0; i < DEVICES_LENGTH; i++) {
    if(DEVICES[i].mac != mac) 
      continue;

    if(DEVICES[i].physical_type == PHYSICAL_SERVO){
      control_servo(DEVICES[i],instruction);  
    }
  
    
    DEVICES[i].latest_status = instruction;
    break;  
  }
};

void control_servo(const DeviceConfiguration &device, const String &instruction){
  servo.attach(device.pin);
  if(instruction == "true") {
    servo.writeMicroseconds(1900); 
    send_status(device.mac,"true");
   }
   else {
    servo.writeMicroseconds(1000);
    send_status(device.mac,"false");
   }
}


/**
 * Sendet den Status eines Geräts per MQTT.
 * @param mac Das Gerät, dessen Status übermittelt wird.
 * @param status Der zu sendende Status des Geräts.
 */
void send_status(const String &mac, const String &payload){
  mqtt_client.publish("status", mac + ":" + payload);
}

/**
 * Sendet eine Registrierung.
 * @param mac Die Mac des zu registrierenden Geräts.
 * @param device_type Der Gerätetyp.
 * @param parking_lot_nr Die optionale Parkplatznummer.
 * @param parent_mac Das optionale übergeordnete Gerät.
 */
void send_register(const String &mac, const DeviceTypeName &device_type, const int &parking_lot_number, const String &parent_mac){
  String parking_lot_number_as_string = parking_lot_number == NULL_INT ? "": String(parking_lot_number);
  String parent_mac_as_string = parent_mac == NULL_STRING ? "" : parent_mac;
  String message = mac + ":" + device_type_name_to_string(device_type) + ":" + parking_lot_number_as_string + ":" + parent_mac_as_string;
  mqtt_client.publish("register", message);
}

/**
 * Eine Nachricht wurde per MQTT empfangen.
 * @param topic Die lane.
 * @param payload Die empfangende Nachricht.
 */
void message_received(String &topic, String &payload) {
  
  if (topic == "scan") {
    reveive_scan();
    Serial.println("received something on scan-lane");
  }
  else if(topic == "instruction") {
    int payload_finder = payload.indexOf(":");
    String mac = payload.substring(0, payload_finder);
    String instruction = payload.substring(payload_finder+1);
    receive_instruction(mac,instruction);
    Serial.println("received on instruction-lane for mac: " + mac + " the instruction: " + instruction);
  }
}

/**
 * Prüft ob die Gerätekonfiguration gültig ist.
 * @return Gibt true zurück, sollte die Konfiguration stimmen, ansonten false.
 */
boolean is_device_configuration_valid(){
  Serial.print("Check device configurations: ");
   
  for(int i = 0; i < DEVICES_LENGTH; i++) 
  {
    const DeviceConfiguration &lhs = DEVICES[i];
    for(int y = 0; y < DEVICES_LENGTH; y++){
      const DeviceConfiguration &rhs = DEVICES[y];
      if(y == i) 
        continue;
      // Prüfe ob Mac unique
      if(rhs.mac == lhs.mac){
        Serial.println("Invalid, mac: " + lhs.mac +  " is not unique.");
        return false;
      }
    }
    // Prüfe ob pin gültig
    if(lhs.pin <= 0){
       Serial.println("Invalid, device with mac: " + lhs.mac +  " has invalid pin.");
       return false;
    }
  }
  Serial.println("done");
  return true;
}

/**
 * Konfiguriert den Arduino für alle Geräte.
 * Sollte die Gerätekonfiguration ungültig sein, wird das Programm beendet.
 */
void setup_devices(){
  //if(!is_device_configuration_valid()) 
  //  exit(1);
  // Registriere Geräte
  for(int i = 0; i < DEVICES_LENGTH; i++){
    pinMode(DEVICES[i].pin, DEVICES[i].is_input ?  INPUT : OUTPUT);
    if(DEVICES[i].physical_type == PHYSICAL_SERVO) {
      control_servo(DEVICES[i],"false");
    }
  }
    
}

/**
 * Konfiguriert das WiFi-Modul und stellt eine Verbindung zum WLAN her.
 */
void setup_wifi(){
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
}

/**
 * Initialisiert den MQTT-Client.
 */
void setup_mqtt(){
  mqtt_client.begin(MQTT_BROKER, wifi_client);
  mqtt_client.onMessage(message_received);
  connect_mqtt();
}

/**
 * Verbindet den initialsierten MQTT-Client mit dem MQTT-Broker.
 */
void connect_mqtt() {
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

void setup() {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  setup_devices();
  setup_wifi();
  setup_mqtt();
}

void loop() {
  if (!mqtt_client.connected()) 
    connect_mqtt();
  mqtt_client.loop();

  // TODO: Geräte updates senden
  // Kleiner Test
  /*
  int rand = random(10000);
  if(rand <= 1){
    int deviceIndex = random(DEVICES_LENGTH);
    send_status(DEVICES[deviceIndex].mac, String(random(10)));  
  }
  */
}
