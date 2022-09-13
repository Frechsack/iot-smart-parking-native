  #include <MQTT.h>
#include <SPI.h>
#include <WiFiNINA.h>
#include "properties.h"
#include <Arduino.h>
#include <Servo.h>
#include <Adafruit_NeoPixel.h>
#include <Wire.h>
#include "Adafruit_SGP30.h"
#include "Adafruit_LEDBackpack.h"
#include "Adafruit_LEDBackpack.h"

/**
 * Die logischen Gerätetypen.
 */
enum LogicalType {
  ENTER_BARRIER, EXIT_BARRIER, LAMP, PARKING_GUIDE_LAMP, CWO_SENSOR, MOTION_SENSOR, SPACE_DISPLAY, SPACE_ENTER_LIGHT, SPACE_EXIT_LIGHT, ALARM
};

/**
 * Die physischen Gerätetypen.
 */
enum PhyisicalType {
  SERVO, ADAFRUIT_NEOPIXEL, MH_SERIES_WIRE_SENSOR, ADAFRUIT_SGP30, ADAFRUIT_7_SEGMENT_DISPLAY, SIMPLE
};

/**
 * Stellt ein virtuelles Gerät da.
 */
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
   * Der verwendete Pin des Geräts. Sollte ein Gerät ohne Pinsteuerung auskommen, kann -1 verwendet werden.
   */
  int pin;

  /**
   * Der optionale Identifier des Geräts. Dient um ein virtuelles Gerät auf einem Physichen Gerät zu identifizieren. 
   * Verwendet z.B. bei 7-Segment Display oder Neopixel-Band um die Position bzw. Segment zu bestimmen. 
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
   * Die Neopixel steuerung, sollte das Gerät auf einem Neopixelband sitzen. Wird bei Programmstart automatisch initialisiert.
   */
  Adafruit_NeoPixel* neopixel;
  
  /**
   * Sie Servo-Setuerung, sollte das Gerät ein Servomotor sein. Wird bei Programmstart automatisch initialisiert.
   */
  Servo* servo;

  /**
   * Die Anzahl an Pixeln auf dem Neopixelband.
   */
  int* neopixel_pixel_count;

  /**
   * Die Adresse eines 7-Segment Displays. Der Standart ist 0x70 für das Adafruit 7-Segment-Display.
   */
  int* address;

  /**
   * Die Display-Steuerung, sollte es sich bei dem Gerät um ein 7-Segment-Display handeln. Wird bei Programmstart automatisch initialisiert.
   */
  Adafruit_7segment* display;
};



/**
 * Die Anzahl an virtuellen Geräten.
 */
const int DEVICES_LENGTH = 14;

/**
 * Die Virtuellen Geräte.
 */
Device DEVICES[DEVICES_LENGTH] = {
  /*MAC       LOGICAL_TYPE        PHYSICAL_TYPE                   PIN   IDENTIFIER    PARKING_LOT_NR  PARENT_MAC              LATEST_STATUS  NEOPIXEL  SERVO  NEOXPIXEL_PIXEL_COUNT  ADDRESS        DISPLAY*/
  /*{"DSP_1"  , SPACE_DISPLAY       , ADAFRUIT_7_SEGMENT_DISPLAY  , -1  , new int(0)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  */{"PGL_1"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(0)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_2"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_3"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(2)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_4"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(3)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_5"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(4)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_6"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(5)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_7"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(6)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_8"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(7)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_9"  , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(8)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL},
  {"PGL_10" , PARKING_GUIDE_LAMP    , ADAFRUIT_NEOPIXEL           , 11   , new int(9)  , NULL          , NULL                  , NULL        , NULL    , NULL  , new int(20)         , new int(0x70) , NULL}, 
  /*{"DIS_1"  , SPACE_DISPLAY         , ADAFRUIT_7_SEGMENT_DISPLAY  , -1  , new int(0)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  /*{"WS_1"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 34  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  {"WS_2"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 35  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  {"WS_3"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 33  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL},
  {"WS_4"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 36  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  {"WS_5"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 32  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL},
  {"WS_6"   , MOTION_SENSOR         , MH_SERIES_WIRE_SENSOR       , 37  , new int(1)  , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, */ 
  {"CWO "   , CWO_SENSOR            , ADAFRUIT_SGP30              , -1  , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  //{"ENT "   , ENTER_BARRIER         , SERVO                       , 30   , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL},
  //{"EXT "   , EXIT_BARRIER          , SERVO                       , 31   , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , new int(0x70) , NULL}, 
  {"SPL_1"   , SPACE_EXIT_LIGHT           , SIMPLE                       , 6   , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , NULL , NULL},   
  {"ALR"   , ALARM           , SIMPLE                       , 7   , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , NULL , NULL},
  {"SPL_2"   , SPACE_ENTER_LIGHT           , SIMPLE                       , 5   , NULL        , NULL          , NULL                  , NULL        , NULL    , NULL  , NULL                , NULL , NULL},    
};


WiFiClient wifi_client;
MQTTClient mqtt_client;

/**
 * Der Co2-Sensor. Pro Board kann nur ein solcher Sensor angeschlossen werden, daher global deklariert und nicht per Co2-Sensor.
 */
const Adafruit_SGP30 cwo_sensor;

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
  if(type == MOTION_SENSOR)
    return "MOTION_SENSOR";
  if(type == SPACE_DISPLAY)
    return "SPACE_DISPLAY";
  if(type == SPACE_ENTER_LIGHT)
    return "SPACE_ENTER_LIGHT";
  if(type == SPACE_EXIT_LIGHT)
    return "SPACE_EXIT_LIGHT";
  Serial.print("LogicalType forgot to be implemented, type: \"" );
  Serial.print(type);
  Serial.println("\"");
  delay(2000);
  exit(0);
};

/**
 * Es wurde eine "instruction"-Anweisung per MQTT erhalten.
 * @param mac Das Gerät, welches geschalten werden soll.
 * @param instruction Der neue anzunehmende Zustand.
 */
void process_instruction(const String &mac, const String &instruction){
  // Prüfe ob Gerät auf diesem Board
   for(int i = 0; i < DEVICES_LENGTH; i++) {
    Device &device = DEVICES[i];
    if(device.mac != mac) 
      continue;
    // Abhängig von Gerätetyp schalten
    if(device.logical_type == ENTER_BARRIER || device.logical_type == EXIT_BARRIER)
      instruct_servo(device,instruction);  
    else if(device.physical_type == ADAFRUIT_NEOPIXEL)
      instruct_adafruit_neopixel(device,instruction);
    else if(device.physical_type == ADAFRUIT_7_SEGMENT_DISPLAY)
      instruct_adafruit_7_segment_display(device,instruction);
     else if(device.physical_type == SIMPLE){
      instruct_simple(device,instruction);
    }
      
    break;  
  }
};

/**
 * Gibt Anweisung an einen Servo zu schalten.
 * @param device Das zu schaltende Gerät.
 * @þaram instruction Der anzunehmende Zustand.
 */
void instruct_servo(Device &device, const String &instruction){
  if(instruction == "true") {
    device.servo->write(40); 
    send_status(device.mac,"true");
    if(device.latest_status == NULL)
      device.latest_status = new String("true");
    else
      *device.latest_status = "true";
   }
   else if(instruction == "false") {
    device.servo->write(130);
    send_status(device.mac,"false");
     if(device.latest_status == NULL)
      device.latest_status = new String("false");
    else
      *device.latest_status = "false";
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
void instruct_adafruit_neopixel(Device &device, const String &instruction){
  if(instruction == "true") {
    // TODO: Alte Zustände wieder reinladen
    device.neopixel->setPixelColor(*device.identifier,device.neopixel->Color(100,100,100));
    device.neopixel->show();
    send_status(device.mac,"true");
    if(device.latest_status == NULL)
      device.latest_status = new String("true");
    else
      *device.latest_status = "true";
   }
   else if(instruction == "false") {
    device.neopixel->setPixelColor(*device.identifier,device.neopixel->Color(0,0,0));
    device.neopixel->show();
    send_status(device.mac,"false");
    if(device.latest_status == NULL)
      device.latest_status = new String("false");
    else
      *device.latest_status = "false";
   }
   else {
    Serial.println("Error: Invalid instruction for device, type: \"ADAFRUIT_NEOPIXEL\", instruction: \"" + instruction +  "\"");
   }
}

/**
 * Gibt Anweisung an ein 7-Segment Display.
 * @param device Das zu schaltende Gerät.
 * @þaram instruction Der anzunehmende Zustand.
 */
void instruct_adafruit_7_segment_display(Device &device, const String &instruction){
  if(instruction != NULL && instruction.length() == 1 && isDigit(instruction[0])){
    const int instruction_as_int = instruction.toInt();

    // Schreibe die Ziffer dieses Segments
    device.display->writeDigitNum(*device.identifier,instruction_as_int);

    // Achtung: WriteDisplay löscht den Buffer für jedes nicht gesetzte Segment. Daher müssen zuvor alle Segmente neu geschrieben werden.
    // Iteriere über alle Geräte, welche ein Segment darstellen, nicht dem Gerät dieses Aufrufs entsprechen (mac), auf dem Selben Display sind (address) und einen status haben.
    // Für diese Geräte muss der letzte Status erneut auf dem Display dargestellt werden.
    for(int i = 0; i < DEVICES_LENGTH; i++)
      if(DEVICES[i].physical_type == ADAFRUIT_7_SEGMENT_DISPLAY && DEVICES[i].mac != device.mac && *DEVICES[i].address == *device.address && DEVICES[i].latest_status != NULL)
        device.display->writeDigitNum(*DEVICES[i].identifier,DEVICES[i].latest_status->toInt());

    device.display->writeDisplay();
    if(device.latest_status == NULL) 
      device.latest_status = new String(instruction_as_int);
    else 
      *device.latest_status = String(instruction_as_int);
    send_status(device.mac,*device.latest_status);
  }
  else {
    Serial.println("Error: Invalid instruction for device, type: \"ADAFRUIT_7_SEGMENT_DISPLAY\", instruction: \"" + instruction +  "\"");
  }
}

void instruct_simple(Device &device, const String &instruction){
  boolean is_enabled = instruction == "true";
  
  if(is_enabled) {
    digitalWrite(device.pin,LOW);
    String* latest_status = device.latest_status == NULL ? new String() : device.latest_status;
    *latest_status = "true";
    device.latest_status = latest_status; 
    send_status(device.mac,"true");
 }
  else{
    digitalWrite(device.pin,HIGH);
    String* latest_status = device.latest_status == NULL ? new String() : device.latest_status;
    *latest_status = "false";
    device.latest_status = latest_status; 
        send_status(device.mac,"false");
  }
}

/**
 * Synchronisiert ein virtuelles Gerät. Hat das Gerät einen neuen Zustand angenommen, wird dieser per MQTT übermittelt.
 * @param device Das zu synchronisierende Gerät.
 */
void synchronize_adafruit_sgp30(Device &device){
  // Prüfe ob Messung erfolgreich
  if(!cwo_sensor.IAQmeasure()){
    return;
  }
  int co2 = cwo_sensor.eCO2;
  co2 = (int ) (co2 / 300);
  if(device.latest_status == NULL) {
    device.latest_status = new String(co2);
    send_status(device.mac,*device.latest_status);
    return; 
  }

  String co2_as_string = String(co2);
  if(*device.latest_status != co2_as_string) {
    *device.latest_status = co2_as_string;
    send_status(device.mac,co2_as_string);
  }
}

/**
 * Synchronisiert ein virtuelles Gerät. Hat das Gerät einen neuen Zustand angenommen, wird dieser per MQTT übermittelt.
 * @param device Das zu synchronisierende Gerät.
 */
void synchronize_mh_series_wire_sensor(Device& device){
  const boolean value = digitalRead(device.pin) == LOW;
  // Prüfe ob Sensor neuen Wert angenommen hat.
  //if(device.latest_status != NULL) Serial.println(*device.latest_status);
  //Serial.println(value);
  if(device.latest_status == NULL || (*device.latest_status == "true" && value == false) || (*device.latest_status == "false" && value)) {
    delete device.latest_status; 
    device.latest_status = new String(value ? "true" : "false");
    send_status(device.mac, *device.latest_status);
  }
}

/**
 * Sendet den Status eines Geräts per MQTT.
 * @param mac Das Gerät, dessen Status übermittelt wird.
 * @param status Der zu sendende Status des Geräts.
 */
void send_status(const String &mac, const String &status){
  Serial.println("Send status, message: \""+ mac + ":" + status + "\"");
  mqtt_client.publish("status", mac + ":" + status);
}

/**
 * Sendet eine Registrierung.
 * @param mac Die Mac des zu registrierenden Geräts.
 * @param device_type Der Gerätetyp.
 * @param parking_lot_nr Die optionale Parkplatznummer.
 * @param parent_mac Das optionale übergeordnete Gerät.
 */
void send_register(const String &mac, const LogicalType &type, const int* parking_lot_number, const String* parent_mac){
  String parking_lot_number_as_string = parking_lot_number == NULL ? "": String(*parking_lot_number);
  String parent_mac_as_string = parent_mac == NULL ? "" : *parent_mac;
  String message = mac + ":" + logical_type_to_string(type) + ":" + parking_lot_number_as_string + ":" + parent_mac_as_string;
  Serial.println("Send register, message: \"" + message + "\"");
  mqtt_client.publish("register", message);
}

/**
 * Es wurde eine "scan"-Anweisung per MQTT erhalten.
 */
void process_scan(){
  // Verschicke zuerst nur jene ohne parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac == NULL){
      send_register(DEVICES[i].mac,DEVICES[i].logical_type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac); 
    }
  delay(2000);
  // Dann jene mit parent
  for(int i = 0; i < DEVICES_LENGTH; i++)
    if(DEVICES[i].parent_mac != NULL)
      send_register(DEVICES[i].mac,DEVICES[i].logical_type,DEVICES[i].parking_lot_nr, DEVICES[i].parent_mac);   
  
};

/**
 * Eine Nachricht wurde per MQTT empfangen.
 * @param topic Die lane.
 * @param payload Die empfangende Nachricht.
 */
void message_received(String &topic, String &payload) {
  if (topic == "scan") {
    Serial.println("Received scan-lane");
    process_scan();
  }
  else if(topic == "instruction") {
    int payload_finder = payload.indexOf(":");
    String mac = payload.substring(0, payload_finder);
    String instruction = payload.substring(payload_finder+1);
    Serial.println("Received instruction-lane, mac: \"" + mac + "\" instruction: \"" + instruction + "\"");
    process_instruction(mac,instruction);
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
    if(lhs.physical_type == SERVO) {
      if(lhs.pin <= 0){
        Serial.println("failed: device: " + lhs.mac + " has invalid pin.");
        delay(2000);
        exit(1);
      }
    }
    else if(lhs.physical_type == ADAFRUIT_NEOPIXEL){
      if(lhs.identifier == NULL || lhs.neopixel_pixel_count == NULL){
        Serial.println("failed: device: " + lhs.mac + " has no identifier or pixel-count.");
        delay(2000);
        exit(1);
      } 
    }
     else if(lhs.physical_type == ADAFRUIT_7_SEGMENT_DISPLAY){
      if(lhs.address == NULL || lhs.identifier == NULL){
        Serial.println("failed: device: " + lhs.mac + " has no address or identifier.");
        delay(2000);
        exit(1);
      } 
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
  if(!is_device_configuration_valid()){
    delay(2000);
    exit(1);
  }
  
  // Konfiguriere Geräte 
  for(int i = 0; i < DEVICES_LENGTH; i++){
    Device &device = DEVICES[i];
    // Servo
    if(device.physical_type == SERVO){
      pinMode(device.pin,OUTPUT);
      device.servo = new Servo();
      device.servo->attach(device.pin);
      
      // Standartzustand
      instruct_servo(DEVICES[i],"false");
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
        pinMode(device.pin,OUTPUT);
        neopixel = new Adafruit_NeoPixel(*device.neopixel_pixel_count,device.pin);
        neopixel->begin();
        neopixel->setBrightness(100);
      }
      device.neopixel = neopixel;
      // Standartzustand
      instruct_adafruit_neopixel(device,"false"); 
    }
    else if(device.physical_type == MH_SERIES_WIRE_SENSOR){
      // Keine Konfig notwendig
    }
    else if(device.physical_type == ADAFRUIT_7_SEGMENT_DISPLAY) {
      // Die einzelnen Segmente teilen sich einen Controller, prüfe ob ein solcher bereits vorher erstellt wurde.
      Adafruit_7segment* display = NULL;
      for(int y = 0; y < i; y++) 
        if(DEVICES[y].physical_type == ADAFRUIT_7_SEGMENT_DISPLAY && DEVICES[y].address == device.address){
          display = DEVICES[y].display;
          break;
        } 
      // Erstelle neuen Controller, sollte der Controller noch nicht existieren
      if(display == NULL){
        display = new Adafruit_7segment();
        display->begin(*device.address);
      }
      device.display = display;
      // Standartzustand
      device.display->clear();
    }
    else if(device.physical_type == SIMPLE){
      pinMode(device.pin,OUTPUT);
      instruct_simple(device,"false");     
    }
  }
}

/**
 * Konfiguriert das WiFi-Modul und stellt eine Verbindung zum WLAN her. 
 * Konnte das WiFi-Module nicht konfiguriert werden, bricht das Programm ab.
 * Blockt solange, bis eine WLAN Verbindung aufgebaut ist.
 */
void setup_wifi(){
  Serial.print("Setup Wifi-Module: ");
  // Registriere WiFi-Modul
  WiFi.setPins(SPIWIFI_SS, SPIWIFI_ACK, ESP32_RESETN, ESP32_GPIO0, &SPIWIFI);
  // Verzögerung zum starten des Moduls
  for(int i = 0; i < 10 && WiFi.status() == WL_NO_MODULE; i++)
    delay(1000);
  if(WiFi.status() == WL_NO_MODULE){
    Serial.println("failed");
    delay(2000);
    exit(1);
  }
  else
    Serial.println("done");
    
  Serial.print("Connect to WiFI: ");
  Serial.print(WIFI_SSID);
  Serial.print(": ");

  // WiFi Verbinden
  int wifi_status = WL_IDLE_STATUS;
  do {
    wifi_status = WiFi.begin(WIFI_SSID, WIFI_PASS);
    delay(100);
  } while (wifi_status != WL_CONNECTED);
  Serial.println("done");
}

/**
 * Konfiguriert den MQTT-Client.
 */
void setup_mqtt(){
  Serial.print("Setup Mqtt-Client: ");
  mqtt_client.begin(MQTT_BROKER, wifi_client);
  mqtt_client.onMessage(message_received);
  Serial.println("done");
  //connect_mqtt();
}

/**
 * Konfiguriert den Co2-Sensor.
 * Konnte der Sensor nicht konfiguriert werden, bricht das Programm ab.
 * Blockiert solange, bis der Sensor konfiguriert wurde.
 */
void setup_cwo_sensor(){
  Serial.print("Setup Cwo-Sensor: ");
  int count = 0;
  boolean is_valid = false;
  do
     if(cwo_sensor.begin()){
      is_valid = true;
      break;
     }
     else
      delay(1000);
  while(count++ <= 10);
  
  if(is_valid)
    Serial.println("done");
  else {
    Serial.println("failed");
    delay(2000);
    exit(0);
  }
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
  
  setup_wifi();
  setup_mqtt();
  connect_mqtt();
  setup_devices();
  setup_cwo_sensor();
}

void loop() {
  
  if (!mqtt_client.connected()) 
    connect_mqtt();
  mqtt_client.loop();

  for(int i = 0; i < DEVICES_LENGTH; i++) 
      // Linien-Sensoren
      if(DEVICES[i].physical_type == MH_SERIES_WIRE_SENSOR)
        synchronize_mh_series_wire_sensor(DEVICES[i]);
      else if(DEVICES[i].physical_type == ADAFRUIT_SGP30)
        synchronize_adafruit_sgp30(DEVICES[i]);
  // TODO: Geräte updates senden*/
  // Kleiner Test
  /*
  int rand = random(10000);
  if(rand <= 1){
    int deviceIndex = random(DEVICES_LENGTH);
    send_status(DEVICES[deviceIndex].mac, String(random(10)));  
  }
  */
}
