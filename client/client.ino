#include <MQTT.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "properties.h"
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>

/**
 * Die verfügbaren logischen Gerätetypen.
 */
enum LogicalType {
  ENTER_BARRIER, EXIT_BARRIER, LAMP, PARKING_GUIDE_LAMP, CWO_SENSOR, MOTION_SENSOR
};

/**
 * Die tatsächlich verwendeten Geräte.
 */
enum PhyisicalType {
  SERVO, ADAFRUIT_NEOPIXEL
};


struct Device {
  
  /**
   * Die mac des Geräts.
   */
  String mac;
  
  /**
   * Der abstrakte Gerätetyp.
   */
  LogicalType logical_type;
  
  /**
   * Der physische Gerätetyp.
   */
  PhyisicalType physical_type;

  /**
   * Der verwendete Pin des Geräts.
   */
  int pin;

  /**
   * Der optionale Identifier des Geräts.
   */
  int* identifier;
  
  /**
   * Die optionale Parkplatzzuordnung.
   */
  int* parking_lot_nr;
  /**
   * Das optionale Elternelement.
   */
  String* parent_mac;

  /**
   * Der letzte Status dieses Geräts.
   */
  String* latest_status;
  
  /**
   * Die Neopixel steuerung, sollte das Gerät auf einem Neopixelband sitzen.
   */
  Adafruit_NeoPixel* neopixel;
  
  /**
   * Sie Servo-Setuerung, sollte das Gerät ein Servomotor sein.
   */
  Servo* servo;

  /**
   * Die Anzahl an Pixeln auf dem Neopixelband.
   */
  int* neopixel_pixel_count;
};

/**
 * Die Anzahl an virtuellen Geräten.
 */
const int DEVICES_LENGTH = 3;

/**
 * Die Virtuellen Geräte.
 */
Device DEVICES[DEVICES_LENGTH] = {
  {"PG1",PARKING_GUIDE_LAMP,ADAFRUIT_NEOPIXEL,6,new int(1),NULL,NULL,NULL,NULL,NULL,new int(20)},
  {"PG2",PARKING_GUIDE_LAMP,ADAFRUIT_NEOPIXEL,6,new int(2),NULL,new String("PG1"),NULL,NULL,NULL,new int(20)},
  {"PG3",PARKING_GUIDE_LAMP,ADAFRUIT_NEOPIXEL,6,new int(3),NULL,new String("PG2"),NULL,NULL,NULL,new int(20)}
  //{"S1",ENTER_BARRIER,SERVO,46,NULL,NULL,NULL,NULL,NULL,NULL}
};


int wifi_status = WL_IDLE_STATUS;
WiFiClient wifi_client;
MQTTClient mqtt_client;

/**
 * Konvertiert ein enum des Typs "DeviceTypeName" in einen String.
 * @param type Der zu konvertierende Typ.
 * @returns Gibt den entsprechenden String zurück.
 */
String logical_type_to_string (const LogicalType& type) {
  if (type == ENTER_BARRIER) 
    return "ENTER_BARRIER";
  if (type == EXIT_BARRIER)
    return "EXIT_BARRIER";
  if (type == LAMP)
    return "LAMP";
  if (type == PARKING_GUIDE_LAMP)
    return "PARKING_GUIDE_LAMP";
  if (type == CWO_SENSOR)
    return "CWO_SENSOR";
  return "MOTION_SENSOR"; 
};

/**
 * Es wurde eine "scan"-Anweisung per MQTT erhalten.
 */
void reveive_scan(){
  // Verschicke zuerst nur jene ohne parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac == NULL)
      send_register(DEVICES[i].mac,DEVICES[i].logical_type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac); 
  delay(2000);
  // Dann jene mit parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac != NULL)
      send_register(DEVICES[i].mac,DEVICES[i].logical_type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac);   
  
};

/**
 * Es wurde eine "instruction"-Anweisung per MQTT erhalten.
 * @param mac Das Gerät, welches geschalten werden soll.
 * @param instruction Der neue anzunehmende Zustand.
 */
void receive_instruction(const String &mac, const String &instruction){
  // Prüfe ob Gerät auf diesem Board
   for(int i = 0; i < DEVICES_LENGTH; i++) {
    Device &device = DEVICES[i];
    if(device.mac != mac) 
      continue;
    // Abhängig von Gerätetyp schalten
    if(device.logical_type == ENTER_BARRIER || device.logical_type == EXIT_BARRIER  ){
      servo_instruction(device,instruction);  
    }
    else if(device.physical_type == ADAFRUIT_NEOPIXEL){
      neopixel_instruction(device,instruction);
    }

    delete device.latest_status;
    device.latest_status = new String(instruction);
    break;  
  }
};

/**
 * Gibt Anweisung an einen Servo zu schalten.
 * @param device Das zu schaltende Gerät.
 * @þaram instruction Der anzunehmende Zustand.
 */
void servo_instruction(const Device &device, const String &instruction){
  if(instruction == "true") {
    device.servo->writeMicroseconds(1900); 
    send_status(device.mac,"true");
   }
   else if(instruction == "false") {
    device.servo->writeMicroseconds(1000);
    send_status(device.mac,"false");
   }
   else {
    Serial.println("Error: Invalid instruction for device, type: \"SERVO\", instruction: \"" + instruction +  "\"");
   }
}

/**
 * Gibt Anweisung an eine Neopixel Lampe zu schalten.
 * @param device Das zu schaltende Gerät.
 * @þaram instruction Der anzunehmende Zustand.
 */
void neopixel_instruction(const Device &device, const String &instruction){
  if(instruction == "true") {
    device.neopixel->setPixelColor(*device.identifier,device.neopixel->Color(100,100,100));
    device.neopixel->show();
    send_status(device.mac,"true");
   }
   else if(instruction == "false") {
    device.neopixel->setPixelColor(*device.identifier,device.neopixel->Color(0,0,0));
    device.neopixel->show();
    send_status(device.mac,"false");
   }
   else {
    Serial.println("Error: Invalid instruction for device, type: \"ADAFRUIT_NEOPIXEL\", instruction: \"" + instruction +  "\"");
   }
}


/**
 * Sendet den Status eines Geräts per MQTT.
 * @param mac Das Gerät, dessen Status übermittelt wird.
 * @param status Der zu sendende Status des Geräts.
 */
void send_status(const String &mac, const String &status){
  mqtt_client.publish("status", mac + ":" + status);
}

/**
 * Sendet eine Registrierung.
 * @param mac Die Mac des zu registrierenden Geräts.
 * @param device_type Der Gerätetyp.
 * @param parking_lot_nr Die optionale Parkplatznummer.
 * @param parent_mac Das optionale übergeordnete Gerät.
 */
void send_register(const String &mac, const LogicalType &type, const int &parking_lot_number, const String &parent_mac){
  String parking_lot_number_as_string = parking_lot_number == NULL ? "": String(parking_lot_number);
  String parent_mac_as_string = parent_mac == NULL ? "" : parent_mac;
  String message = mac + ":" + logical_type_to_string(type) + ":" + parking_lot_number_as_string + ":" + parent_mac_as_string;
  mqtt_client.publish("register", message);
}

/**
 * Eine Nachricht wurde per MQTT empfangen.
 * @param topic Die lane.
 * @param payload Die empfangende Nachricht.
 */
void message_received(String &topic, String &payload) {
  if (topic == "scan") {
    Serial.println("Received scan-lane");
    reveive_scan();
  }
  else if(topic == "instruction") {
    int payload_finder = payload.indexOf(":");
    String mac = payload.substring(0, payload_finder);
    String instruction = payload.substring(payload_finder+1);
    Serial.println("Received instruction-lane, mac: \"" + mac + "\" instruction: \"" + instruction + "\"");
    receive_instruction(mac,instruction);
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
    const Device &lhs = DEVICES[i];
    for(int y = 0; y < DEVICES_LENGTH; y++){
      const Device &rhs = DEVICES[y];
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
  if(!is_device_configuration_valid()) 
    exit(1);
  
  // Konfiguriere Geräte 
  for(int i = 0; i < DEVICES_LENGTH; i++){
    Device &device = DEVICES[i];

    // Servo
    if(device.physical_type == SERVO){
      pinMode(device.pin,INPUT);
      device.servo = new Servo();
      device.servo->attach(device.pin);
      // Standartzustand
      servo_instruction(DEVICES[i],"false");
    }
    // Adafruit Neopixel
    else if(device.physical_type == ADAFRUIT_NEOPIXEL){
      // Die einzelnen Lampen teilen sich einen Controller, prüfe ob ein solcher bereits vorher erstellt wurde.
      Adafruit_NeoPixel* neopixel = NULL;
      for(int y = 0; y < i; y++) {
        const Device &compare = DEVICES[y];
        if(compare.physical_type == ADAFRUIT_NEOPIXEL && compare.pin == device.pin){
          neopixel = compare.neopixel;
          break;
        } 
      }
      // Erstelle neuen Controller und konfiguriere Pins, sollte der Controller noch nicht existieren
      if(neopixel == NULL) {
        pinMode(device.pin,INPUT);
        neopixel = new Adafruit_NeoPixel(*device.neopixel_pixel_count,device.pin);
        neopixel->begin();
        neopixel->setBrightness(100);
      }

      device.neopixel = neopixel;
    
      // Standartzustand
      neopixel_instruction(device,"false");
      
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
