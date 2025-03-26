#include <Arduino.h>
#include <SoftwareSerial.h>
#include "gps.h"
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>

SoftwareSerial gpsSerial(0, 1);
GPS gps;


bool hasFix = false;
unsigned long lastFixPrint = 0;
unsigned long lastStatusPrint = 0;
const unsigned long fixInterval = 10000;    // 10 seconds
const unsigned long statusInterval = 2000;  // Status every 2 seconds

#define PRIMARY_RX_PIN 7
#define PRIMARY_TX_PIN 8

#define SECONDARY_RX_PIN 17
#define SECONDARY_TX_PIN 18

#define BUZZER_PIN 99

bool configFlag = false;

int buzzDuration = 0.0f;
unsigned long buzzStart=0;

SoftwareSerial primaryAntenna(PRIMARY_RX_PIN,PRIMARY_TX_PIN);
SoftwareSerial secondaryAntenna(SECONDARY_RX_PIN,SECONDARY_TX_PIN);

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


void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);
  gps.begin(gpsSerial);

  primaryAntenna.begin(115200);
  secondaryAntenna.begin(115200);

  while(!Serial && millis() < 10000); //Wait up to 10 seconds for serial to connect/monitor to be opened
  
}

void loop() {
  gps.update();
  buzzUpdate();
  primaryAntenna.update();
}
