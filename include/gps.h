#ifndef GPS_H
#define GPS_H
#include <Arduino.h>
#include <array>

class GPS{
    public:
        void begin(Stream& serial, uint32_t baud = 9600);
        void update();
        bool hasFix();
        std::array<float,3> getPosition(); //lat, lon, elevation...  get from GGA
        std::array<float,2> getVelocity(); //velocity knots, track degrees... from RMC
        std::array<float,3> getTime(); //hours minutes seconds/milliseconds...  from GGA
        int getFixType(); //0 invalid, 1 gps fix, 2 dgps (3D)...  from GGA
        int getFixCount(); //from GGA
        std::array<float,3> getDOP(); //position dop, horizontal dop, vertical dop...  from GSA
        int getSatelliteCount(); //from GSV

    private:
        void parseNMEALine(const String&line);
        
        float convertToDecimal(const String& raw, const String& dir);

        
        Stream* gpsSerial = nullptr;
        String nmeaBuffer;

        bool fix = false;
        float latitude = 0, longitude = 0, elevation = 0;
        float velocity = 0, track = 0;
        float pdop = 0, hdop = 0, vdop = 0;
        int fixType = 0;
        int fixCount = 0;
        int satelliteCount = 0;
        int hour = 0, minute = 0, second = 0;

    
};
#endif
