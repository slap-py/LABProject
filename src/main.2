#include <SdFat.h>

const uint8_t SD_CS = 10;
SdFat sd;
SdFile file;

void setup() {
  Serial.begin(9600);
  while (!Serial);

  SPI.setMOSI(11);
  SPI.setMISO(12);
  SPI.setSCK(13);
  SPI.begin();

  if (!sd.begin(SD_CS, SD_SCK_MHZ(10))) {
    Serial.println("SdFat init failed!");
    return;
  }

  Serial.println("SdFat init succeeded.");

  // Write test
  if (!file.open("log.txt", O_WRONLY | O_CREAT | O_TRUNC)) {
    Serial.println("Failed to create log.txt");
    return;
  }

  file.println("This is a test log entry.");
  file.println("Another line from Teensy 4.0.");
  file.close();
  Serial.println("Write complete.");

  // Read test
  if (!file.open("log.txt", O_RDONLY)) {
    Serial.println("Failed to open log.txt for reading.");
    return;
  }

  Serial.println("Reading log.txt:");
  int c;
  while ((c = file.read()) >= 0) {
    Serial.write(c);
  }
  file.close();
}

void loop() {}
