#include <Arduino.h>
#include <SoftwareSerial.h>
#include "gps.h"

SoftwareSerial gpsSerial(0, 1);
GPS gps;


bool hasFix = false;
unsigned long lastFixPrint = 0;
unsigned long lastStatusPrint = 0;
const unsigned long fixInterval = 10000;    // 10 seconds
const unsigned long statusInterval = 2000;  // Status every 2 seconds

void setup() {
  Serial.begin(9600);
  gpsSerial.begin(9600);

  gps.begin(gpsSerial);  // Our new custom class just stores the stream

  Serial.println("Booting up... waiting for GPS module.");
}

void loop() {
  gps.update();  // Call this every loop

  if (millis() - lastStatusPrint >= statusInterval) {
    lastStatusPrint = millis();

    int sats = gps.getFixCount();
    Serial.print("Satellites: ");
    Serial.print(sats);

    if (gps.hasFix()) {
      Serial.print(" | FIXED ✅");
      hasFix = true;
    } else {
      Serial.print(" | NO FIX ❌");
      hasFix = false;
      Serial.println();
      Serial.print(gps.getFixType());
      Serial.print("   ");
      Serial.print(gps.getFixCount());
    }

    Serial.println();
  }

  if (hasFix && millis() - lastFixPrint >= fixInterval) {
    lastFixPrint = millis();

    std::array<float, 3> pos = gps.getPosition();
    Serial.print("Lat: ");
    Serial.print(pos[0], 6);
    Serial.print(" | Lon: ");
    Serial.print(pos[1], 6);
    Serial.print(" | Alt: ");
    Serial.println(pos[2], 1);
  }
}
