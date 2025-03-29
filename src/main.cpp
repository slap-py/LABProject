#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

#define gpsSerial Serial1  // Pins 0 (RX) and 1 (TX)

#define primaryAntenna Serial2
#define secondaryAntenna Serial3


bool hasFix = false;
unsigned long lastFixPrint = 0;
unsigned long lastStatusPrint = 0;
const unsigned long fixInterval = 10000;    // 10 seconds
const unsigned long statusInterval = 2000;  // Status every 2 seconds

#define PRIMARY_RX_PIN 7
#define PRIMARY_TX_PIN 8

#define SECONDARY_RX_PIN 16
#define SECONDARY_TX_PIN 17

#define BUZZER_PIN 2

bool configFlag = false;

int buzzDuration = 0;
unsigned long buzzStart=0;

TinyGPSPlus gps;


String get_value(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = {0, -1};
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i+1 : i;
    }
  }

  if (found > index) {
    return data.substring(strIndex[0], strIndex[1]);
  } else {
    return "";
  }
}

void buzz(int frequency, int duration){
  tone(BUZZER_PIN,frequency);
  buzzDuration = duration;
  buzzStart = millis();
}

void buzzUpdate(){
  if ((millis()-buzzStart) >= buzzDuration){
    //stop buzz
    noTone(BUZZER_PIN);
    buzzStart = 0;
    buzzDuration = 0;
  }
}

void _buzzHold(int frequency, int duration){
  tone(BUZZER_PIN,frequency);
  delay(duration);
  noTone(BUZZER_PIN);
}

void log(String message){
  String output = String(millis())+": "+message;
  Serial.println(output);
  //ADD SD CARD LOG FILE INTEGRATION
}

// equipment on the board:
// Teensy 4.0
// BME680
// NEO-7M GPS breakout
// MPU6050
// RYLR890
// RYLR 998
// SD Card logger
// buzzer



void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);

  primaryAntenna.begin(115200);
  secondaryAntenna.begin(115200);

  unsigned long timeout = millis();
  tempmon_init();
  while (!Serial && (millis() - timeout < 5000));

  
  Wire.begin();
  pinMode(BUZZER_PIN,OUTPUT);
  
  _buzzHold(2000,500);

  log("Initializing primary antenna.");
}
unsigned long lastTrigger = 0;
const unsigned long interval = 10000;



void loop() {
  
  buzzUpdate();
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
    
  } 
  

  
  
}
