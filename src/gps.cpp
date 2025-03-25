# include "GPS.h"

std::array<float,3> getPosition(); //lat, lon, elevation...  get from GGA
        std::array<float,2> getVelocity(); //velocity knots, track degrees... from RMC
        std::array<float,3> getTime(); //hours minutes seconds/milliseconds...  from GGA
        int getFixQuality(); //0 invalid, 1 gps fix, 2 dgps (3D)...  from GGA
        int getFixCount(); //from GGA
        std::array<float,3> getDOP(); //position dop, horizontal dop, vertical dop...  from GSA
        int getSatelliteCount(); //from GSV



void GPS::parseNMEALine(const String& line){
    if (line.startsWith("$GPGGA")){
        int idx[15];
        int count = 0;

        for (int i = 0; i < line.length() && count < 15; i++){
            if (line[i] == ','){
                idx[count++] = i;
            }
        }
        String time = line.substring(idx[0]+1,idx[1]);
        hour = time.substring(0,2).toInt();
        minute = time.substring(2,4).toInt();
        second = time.substring(4,6).toInt();

        latitude = convertToDecimal(line.substring(idx[1]+1,idx[2]),line.substring(idx[2]+1,idx[3]));
        longitude = convertToDecimal(line.substring(idx[3]+1,idx[4]),line.substring(idx[4]+1,idx[5]));
        elevation = line.substring(idx[8]+1,idx[9]).toFloat();

        fixType = line.substring(idx[6]+1,idx[7]).toInt();
        fixCount = line.substring(idx[7]+1,idx[8]).toInt();

    } else if (line.startsWith("$GPRMC")){
        int idx[13];
        int count = 0;

        for (int i = 0; i < line.length() && count < 13; i++){
            if (line[i] == ','){
                idx[count++] = i;
            }
        }

        String time = line.substring(idx[0]+1,idx[1]);
        hour = time.substring(0,2).toInt();
        minute = time.substring(2,4).toInt();
        second = time.substring(4,6).toInt();

        String fixStatus = line.substring(idx[1]+1,idx[2]);
        fix = (fixStatus == "A");

        latitude = convertToDecimal(line.substring(idx[2]+1,idx[3]),line.substring(idx[3]+1,idx[4]));
        longitude = convertToDecimal(line.substring(idx[4]+1,idx[5]),line.substring(idx[5]+1,idx[6]));

        velocity = line.substring(idx[6]+1,idx[7]).toFloat();
        track = line.substring(idx[7]+1,idx[8]).toFloat();

    } else if (line.startsWith("$GPGSA")){
        int idx[18];
        int count = 0;
        for (int i = 0; i < line.length() && count < 18; i++){
            if (line[i] == ','){
                idx[count++] = i;
            }
        }
        fixType = line.substring(idx[1]+1,idx[2]).toInt();
        pdop = line.substring(idx[14]+1,idx[15]).toFloat(); 
        hdop = line.substring(idx[15]+1,idx[16]).toFloat(); 
        vdop = line.substring(idx[16]+1,idx[17]).toFloat(); 

    } else if (line.startsWith("$GPGSV")){
        int idx[20];
        int count = 0;
        for (int i = 0; i < line.length() && count < 20; i++){
            if (line[i] == ','){
                idx[count++] = i;
            }
        }
        satelliteCount = line.substring(idx[2]+1, idx[3]).toInt();
    }
}

float GPS::convertToDecimal(const String& raw, const String& dir){
    int degrees;
    float minutes;
    if((dir == "E") || (dir == "W")){
        degrees = raw.substring(0,3).toInt();
        minutes = raw.substring(3).toFloat();
    }else if((dir == "N") || (dir=="S")){
        degrees = raw.substring(0,2).toInt();
        minutes = raw.substring(2).toFloat();
    }else{
        return NAN;
    }

    float decimal = degrees + (minutes / 60.0f);

    if ((dir == "S") || (dir == "W")){
        decimal*= -1.0f;
    }

    return decimal;
}

std::array<float,3> GPS::getPosition(){
    return {latitude, longitude, elevation};
}

std::array<float,2> GPS::getVelocity(){
    return {velocity,track};
}

std::array<float,3> GPS::getTime(){
    return {hour,minute,second};
}

int GPS::getFixType(){
    return fixType;
}

int GPS::getFixCount(){
    return fixCount;
}

std::array<float,3> GPS::getDOP(){
    return {pdop,hdop,vdop};
}

int GPS::getSatelliteCount(){
    return satelliteCount;
}

void GPS::begin(Stream& serial,uint32_t baud){
    gpsSerial = &serial;
}

void GPS::update() {
    while (gpsSerial && gpsSerial->available()) {
        char c = gpsSerial->read();

        if (c == '\n') {
            parseNMEALine(nmeaBuffer);
            Serial.println(nmeaBuffer);
            nmeaBuffer = "";
        } else if (c != '\r') {
            nmeaBuffer += c;
        }
    }
}

bool GPS::hasFix() {
    return fix && fixType > 0 && fixType >= 1 && fixCount > 0 && fixCount < 99;
}
