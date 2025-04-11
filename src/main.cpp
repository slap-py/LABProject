#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME680.h>
#include <Adafruit_BMP280.h> //ADD TO src files
#include <SdFat.h>
#include <SPI.h>

#define gpsSerial Serial1  // Pins 0 (RX) and 1 (TX)

#define primaryAntenna Serial2
#define secondaryAntenna Serial3


bool hasFix = false;
unsigned long lastFixPrint = 0;
unsigned long lastStatusPrint = 0;
const unsigned long fixInterval = 10000;    // 10 seconds
const unsigned long statusInterval = 2000;  // Status every 2 seconds

constexpr double PI = 3.1415927;

SdFat sd;
SdFile logfile;

#define PRIMARY_RX_PIN 7
#define PRIMARY_TX_PIN 8

#define SECONDARY_RX_PIN 16
#define SECONDARY_TX_PIN 17

#define BUZZER_PIN 2

bool configFlag = false;

int buzzDuration = 0;
unsigned long buzzStart=0;

int startupTxIndex = 0;
char startupLog[16];

TinyGPSPlus gps;

Adafruit_BME680 bmeExternal;
Adafruit_BMP280 bmeInternal;
Adafruit_MPU6050 mpu;

File logFile;


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



void log(String message, SdFile _logfile){
  String output = String(millis())+": "+message;
  Serial.println(output);

  if(_logfile){
    _logfile.println(output);

  }else{
    Serial.println("LOG function failed to open startuplog.txt .");
  }
}

String sendAT(HardwareSerial& port, const char* cmd) {
  port.print(cmd);
  port.print("\r\n");
  log(cmd);
  delay(300);

  String response = "";
  unsigned long start = millis();
  while (millis() - start < 200) {
    while (port.available()) {
      response += (char)port.read();
    }
  }
  log(response);
  return response;
}

void configureAntenna(HardwareSerial& port, const char* label, const char* command) {
  String response = sendAT(port, command).trim();
  if (response == "+OK") {
    log(String(label) + " acknowledged.");
    if (startupTxIndex < 16) {
      startupLog[startupTxIndex++] = 1;
    }
  } else {
    log(String(label) + " failed.");
    log(response);
    if (startupTxIndex < 16) {
      startupLog[startupTxIndex++] = 0;
    }
  }
}



/*
Startup procedures by Board equipment:
LOG ALL TO SD

Teensy 4.0:
/ Enable temperature monitor
/ Check temperature between 0c and 60c

NEO-7M GPS breakout
/ Connect via module.
/ Stay in setup holding loop until fix is achieved.

/ RYLR890
/ RYLR998

BME680
/ Enable i2c connection via bme68x adafruit module.
/ Check reading
-- 5 minute burn in phase for the gas sensor**


MPU6050
/ Enable i2c connection via MPU6050 adafruit module.
- Check reading.

SD Card logger
- Ensure connection via module.
- Write all startup data, check if written.

buzzer
- Simple test.
*/

//Teensy related definitions
constexpr int serial_connect_wait = 5000; // Maximum time to wait for serial to connect.
constexpr float minimum_chip_temperature = 0;
constexpr float maximum_chip_temperature = 60;
//GPS Related definitions
constexpr unsigned long gps_fix_wait = 1000; // Maximum time waiting for fix
constexpr unsigned long gps_fix_hold = 5000; // Time waiting for the fix to hold.

//Antenna related definitions
constexpr unsigned long antenna_band = 915500000;
constexpr const char* initializationAwakeMessage = "ARGON Embedded v1 SCIENTIFIC LOW-ALTITUDE BALLOON SOFTWARE.";
String initializationAwakePayload = "AT+SEND=2," + String(strlen(initializationAwakeMessage)) + "," + initializationAwakeMessage;


bool GPSFixSuccess = false;

void setup() {
  Serial.begin(115200);
  gpsSerial.begin(9600);
  _buzzHold(2000,500);
  
  unsigned long serial_connect_timeout = millis();
  tempmon_init();

  //SD CARD INIT
  if(!sd.begin(SD_CS,SD_SCK_MHZ(10))){
    log("SD Card initialization failed.");
  }else{
    log("SD Card initialization successful",logfile);
  }

  logfile.open("ARGON_startup_log.txt",O_WRONLY | O_CREAT | O_APPEND);

  //Check Teensy temperature & initialize serial connection.
  while (!Serial && (millis() - serial_connect_timeout < serial_connect_wait));
  float temperature_start = tempmonGetTemp();
  if (temperature_start < maximum_chip_temperature || temperature_start > minimum_chip_temperature){
    log("Teensy 4.0 temperature in range.",logfile);
    startupLog[startupTxIndex] = 1;
  }else{
    log("Teensy 4.0 temperature NOT in range. Allow cooling.",logfile);
    startupLog[startupTxIndex] = 0;
  }
  startupTxIndex++;

  unsigned long beginGPS = millis();
  unsigned long fixStart = 0;

  //GPS Initialization
  log("Beginning GPS initialization.",logfile);
  while ((millis() - beginGPS) < gps_fix_wait){
    while (gpsSerial.available()) {
      gps.encode(gpsSerial.read());
    }
    if(gps.location.isValid()){
      if(fixStart == 0){
        fixStart = millis();
        log("Achieved GPS fix. Starting countdown.",logfile);
      }else if (millis() - fixStart >= gps_fix_hold){
        String gpsPrintString = "GPS fix held | "+String(gps.location.lat())+","+String(gps.location.lng())+" | "+ String(gps.altitude.meters())+"m | Satellites: "+String(gps.satellites.value())+".";
        log(gpsPrintString,logfile);
        GPSFixSuccess = true;
        break;
      }
    }else{
      if(fixStart !=0){
        log("Fix lost. Restarting.",logfile);
        fixStart = 0;
      }
    }
  }
  if(GPSFixSuccess == false){
    startupLog[startupTxIndex] = 0;
  }else{
    startupLog[startupTxIndex] = 1;
  }
  startupTxIndex++;

  //Primary antenna (RYLR998) initialization.
  primaryAntenna.begin(115200);
  
  
  //Secondary antenna (RYLR896) initialization.
  secondaryAntenna.begin(115200);
  
  Wire.begin();
  pinMode(BUZZER_PIN,OUTPUT);
  log("Out of GPS loop.",logfile);
  String primaryATResponse = sendAT(primaryAntenna,"AT").trim();
  if (primaryATResponse=="+OK"){
    log("Primary antenna responsive.",logfile);
    startupLog[startupTxIndex] = 1;
  }else{
    log("Primary antenna not responsive.",logfile);
    startupLog[startupTxIndex] = 0;
    log(primaryATResponse,logfile);
  }
  startupTxIndex++;

  String secondaryATResponse = sendAT(secondaryAntenna,"AT").trim();
  if (secondaryATResponse=="+OK"){
    log("Secondary antenna responsive.\n",logfile);
    startupLog[startupTxIndex] = 1;
  }else{
    log("Secondary antenna not responsive.\n",logfile);
    startupLog[startupTxIndex] = 0;
    log(secondaryATResponse,logfile);
  }
  startupTxIndex++;

  // Primary
  configureAntenna(primaryAntenna, "Primary antenna network id", "AT+NETWORKID=6");
  configureAntenna(primaryAntenna, "Primary antenna parameter", "AT+PARAMETER=9,7,4,12");
  configureAntenna(primaryAntenna, "Primary antenna band", "AT+BAND=915500000");
  configureAntenna(primaryAntenna, "Primary antenna address", "AT+ADDRESS=1");
  configureAntenna(primaryAntenna, "Primary antenna transmit strength", "AT+CRFOP=22");

  Serial.println("");
  // Secondary
  configureAntenna(secondaryAntenna, "Secondary antenna network id", "AT+NETWORKID=6");
  configureAntenna(secondaryAntenna, "Secondary antenna parameter", "AT+PARAMETER=9,7,4,12");
  configureAntenna(secondaryAntenna, "Secondary antenna band", "AT+BAND=915500000");
  configureAntenna(secondaryAntenna, "Secondary antenna address", "AT+ADDRESS=3");
  configureAntenna(secondaryAntenna, "Secondary antenna transmit strength", "AT+CRFOP=15");

  log("Antenna configuration complete.",logfile);
  Serial.println();
  if(!bmeExternal.begin()){
    log("BME680 not connected.",logfile);
    startupLog[startupTxIndex] = 0;
  }else{
    startupLog[startupTxIndex] = 1;
  }
  startupTxIndex++;
  bmeExternal.setTemperatureOversampling(BME680_OS_8X);
  bmeExternal.setHumidityOversampling(BME680_OS_4X);
  bmeExternal.setPressureOversampling(BME680_OS_8X);
  bmeExternal.setIIRFilterSize(BME680_FILTER_SIZE_63);
  bmeExternal.setGasHeater(0, 0); //GAS DISABLED INITIALLY.

  bmeExternal.performReading();
  String printData = "BME680 enabled | "+String(bmeExternal.temperature) + " C | " + String(bmeExternal.pressure) + " Pa | " + String(bmeExternal.humidity) + "%.";
  log(printData,logfile);

  if(!bmeInternal.begin()){
    log("BMP280 not connected.",logfile);
    //ADD 0 to startup log
  }else{
    //ADD 1 to startup log
  }
  //startupTxIndex++ here

  bmeInternal.setSampling(
    Adafruit_BMP280::MODE_NORMAL,
    Adafruit_BMP280::SAMPLING_X16,   // Max temp resolution
    Adafruit_BMP280::SAMPLING_X16,   // Max pressure resolution
    Adafruit_BMP280::FILTER_X4,      // Light smoothing
    Adafruit_BMP280::STANDBY_MS_1000 // 1 second between measurements
  );
}
  

  if(!mpu.begin()){
    log("MPU6050 not connected.",logfile);
    startupLog[startupTxIndex] = 0;
  }else{
    log("MPU6050 enabled.",logfile);
    startupLog[startupTxIndex] = 1;
  }
  startupTxIndex++;
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  String summary = "AWK_SMRY ";
  for (int i = 0; i < startupTxIndex; i++) {
    summary += String((int)startupLog[i]);

  }
  log(summary,logfile);
  String summaryPayload = "AT+SEND=2," + String(summary.length()) + "," + summary;
  //initializationAwakePayload

  //sendAT(secondaryAntenna,"AT+SEND=2,5,HELLO");
  log(sendAT(primaryAntenna, initializationAwakePayload.c_str()),logfile); // confirm STILL getting +READY
  log("Sending Initialization message.",logfile);
  unsigned long start = millis();
  String buffer = "";
  while(true){
    if(millis() - start >= 10000){
      break;
    }
    while(primaryAntenna.available()){
      char c = primaryAntenna.read();
      buffer+=c;
    }
    if(buffer.indexOf("ARGON")!= -1){
      log("Acknowledgement received.");
      break;
    }
  }
  if(buffer.indexOf("ARGON") == -1){
    log("Acknowledgement NOT received in time.",logfile);
  }
  log("Sending status summary.",logfile);
  sendAT(primaryAntenna,summaryPayload.c_str());
  

  logFile.close();
}
unsigned long lastTrigger = 0;
const unsigned long interval = 10000;

int timesPerMinute(float updatesPerMinute){
  return (unsigned long)(60000/updatesPerMinute);
};

int timesPerSecond(float updatesPerSecond){
  return (unsigned long)(1000/updatesPerSecond);
};


struct ReadIntervals {
  unsigned long BME;
  unsigned long MPU;
  unsigned long GPS;
  unsigned long SD;
  unsigned long TX;

  bool operator==(const ReadIntervals& other) const {
    return BME == other.BME && MPU == other.MPU && GPS == other.GPS && SD == other.SD && TX == other.TX;
  }
};

struct BMERecord {
  unsigned long timestamp;
  float temperatureInternal, pressureInternal;
  float teensyTemperature;
  float temperatureExternal, pressureExternal, humidityExternal;
  bool externalValid;
};

struct MPURecord {
  unsigned long timestamp;
  float Ax, Ay, Az;
  float Rx, Ry, Rz;
  float mpuTemp;
};

struct GPSRecord {
  unsigned long timestamp;
  double latitude, longitude, elevation;
  double velocity, course;
  int satellites;
  double hdop;
  bool gpsValid;
};


enum LogState{
  NORMALRATE,
  LOWPOWER,
  LANDED,
  HIGHRATE
};

enum FlightState{ //Advance criteria to RECOVERY state: More than 90 minutes in any given state.
  LAUNCHPAD, //Advance criteria: armed by software ... INTERVALS_LOWPOWER
  LAUNCHPAD_ARMED, //Advance criteria: upward movement detected ... INTERVALS_NORMAL
  ASCENT, //Advance criteria to HIGHABOVE state: elevation>15000ft ... Advance criteria to DESCENT state: more than 200ft below MAX ALTITUDE RECORDED ... INTERVALS_NORMAL
  HIGHABOVE, //Advance criteria: More than 200ft below MAX ALTITUDE RECORDED ... INTERVALS_HIGH
  DESCENT, //Advance criteria: Within 2000ft of EXPECTED LANDING ELEVATION ... INTERVALS_NORMAL
  NEARGROUND, //Advance criteria: 10 seconds without movement ... INTERVALS_HIGH
  RECOVERY //Advance criteria: NONE ... INTERVALS_LANDED
};

//Normal sensor reading intervals.
ReadIntervals INTERVALS_NORMAL = {
  timesPerMinute(2),
  timesPerSecond(20),
  timesPerMinute(6),
  timesPerMinute(1),
  timesPerMinute(1)
};

//Low power mode reading intervals.
ReadIntervals INTERVALS_LOWPOWER = {
  timesPerMinute(1),
  timesPerSecond(2),
  timesPerMinute(2),
  timesPerMinute(0.5),
  timesPerMinute(2)
};

//Landed reading intervals.
ReadIntervals INTERVALS_LANDED = {
  timesPerMinute(1),
  timesPerSecond(1),
  timesPerMinute(10),
  timesPerMinute(0.5),
  timesPerMinute(6)
};

ReadIntervals INTERVALS_HIGH = {
  timesPerMinute(4),
  timesPerSecond(50),
  timesPerMinute(10),
  timesPerMinute(2),
  timesPerMinute(6)
};

FlightState currentState = LAUNCHPAD;
unsigned long stateEnterTime = millis();
ReadIntervals currentIntervals = INTERVALS_LOWPOWER;

//logSensors caching
unsigned long lastBMERead = 0;
unsigned long lastMPURead = 0;
unsigned long lastGPSRead = 0;
unsigned long lastSD = 0;
unsigned long lastTransmit = 0;

//Absolute maximums per write.
const int MAX_BME = 10;
const int MAX_MPU = 100;
const int MAX_GPS = 20;
const int MAX_ERRORS = 5

BMERecord bmeBuffer[MAX_BME];
MPURecord mpuBuffer[MAX_MPU];
GPSRecord gpsBuffer[MAX_GPS];

int bmeIndex = 0, mpuIndex = 0, gpsIndex = 0;

ReadIntervals lastActiveIntervals = currentIntervals;
bool readingSuccess;

void handleFlightState(){
  switch (currentState){
    case LAUNCHPAD:
      
  }
}

void logSensors(ReadIntervals _currentIntervals,unsigned long _now){
  if (_currentIntervals == lastActiveIntervals){
    //log change
  }
  if (_now - lastBMERead >= _currentIntervals.BME){
    lastBMERead = millis(); // Log BME data.
    float temperatureInternal = bmeInternal.readTemperature();
    float pressureInternal = bmeInternal.readPressure();
    float teensyTemperature = tempmonGetTemp();
    if(readingSuccess){
      float temperatureExternal = bmeExternal.temperature;
      float pressureExternal = bmeExternal.pressure;
      float humidityExternal = bmeExternal.humidity
    }else{
      float temperatureExternal = 0f;
      float pressureExternal = 0f;
      float humidityExternal = 0f;
      //LOG to file
    }
    BMERecord& r = bmeBuffer[bmeIndex++];
    if (bmeIndex >= MAX_BME) bmeIndex = 0; //BEGIN OVERWRITING CURRENT BUFFER
    bmeExternal.beginReading();
    r.timestamp = lastBMERead
    r.temperatureInternal = temperatureInternal;
    r.pressureInternal = pressureInternal;
    r.teensyTemperature = teensyTemperature;
    r.temperatureExternal = temperatureExternal;
    r.pressureExternal = pressureExternal;
    r.humidityExternal = humidityExternal;
  }

  if(_now - lastBMERead >= 1500){
    bool readingSuccess = bmeExternal.endReading()
  }

  if (_now - lastMPURead >= _currentIntervals.MPU){ //Log MPU data.
    sensor_event_t a,g, temp;
    mpu.getEvent(&a,&g,&temp); //Does not block>2ms
    MPURecord& r = MPUBuffer[MPUIndex++];
    float Ax = a.acceleration.x;
    float Ay = a.acceleration.y;
    float Az = a.acceleration.z;
    
    float Rx = g.gyro.x*(180/PI);
    float Ry = g.gyro.y*(180/PI);
    float Rz = g.gyro.z*(180/PI);

    float MPUTemp = temp.temperature;

    r.Ax = Ax;
    r.Ay = Ay;
    r.Az = Az;
    r.Rx = Rx;
    r.Ry = Ry;
    r.Rz = Rz;
    r.mpuTemp = MPUTemp;

  }
  if (_now - lastGPSRead >= _currentIntervals.GPS){ //Log GPS data.
    if(gps.location.age() >= 5000){
      log("Invalid GPS reading (>5s)",logfile)
    }else{
      GPSRecord& r = GPSBuffer[GPSBuffer++]

      double latitude = gps.location.lat();
      double longitude = gps.location.lng(); 
      double elevation = gps.altitude.meters();
      double velocity = gps.speed.mph();
      double course = gps.course.deg();
      int satellites = gps.satellites.value();
      double hdop = gps.hdop.hdop();

      r.latitude = latitude;
      r.longitude = longitude;
      r.elevation = elevation;
      r.velocity = velocity;
      r.course = course;
      r.satellites = satellites;
      r.hdop = hdop;
    }
    
  }
  if (_now - lastSD >= _currentIntervals.SD){ //Move data to SD card.
    logfile.flush();
    
  }
  if (_now - lastTransmit >= _currentIntervals.TX){ //TX a packet

  }
  
}

void loop() {
  unsigned long now = millis();

  //Rescue from state stuck, transition to recovery.
  if((now-stateEnterTime) > 90UL * 60UL * 1000UL) { //90 minutes
    //MOVE TO RECOVERY STATE.
  }

  
  buzzUpdate();

  //Append GPS data
  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  } 
  

  
  
}
