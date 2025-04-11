#include "windtables.h"
#include <math.h>

#define MAX_WIND_LAYERS 50 // one per 1000ft

WindLayer windTable[MAX_WIND_LAYERS];

int windTableSize = 0;

const float R = 6378137.0;


void recordLayer(float altitudeMin, float altitudeMax, float velocity, float direction){
    if(windTableSize < MAX_WIND_LAYERS){
        windTable[windTableSize++] = {altitudeMin, altitudeMax, velocity, direction};
    }
}

std::array<float, 2> displacementToLatLng(float latStart, float lngStart, float displacement, float direction){
    float displacement_x_m = cos((direction*M_PI / 180.0)) * displacement;
    float displacement_y_m = sin((direction*M_PI / 180.0)) * displacement;

    float deltaLat = (displacement_y_m / R) * (180.0 / M_PI);
    float deltaLng = (displacement_x_m / (R*cos(latStart * M_PI / 180.0)))* (180 / M_PI);

    float latEnd = latStart+deltaLat;
    float lngEnd = lngStart+deltaLng;
    return { latEnd,lngEnd };
}

std::array<float,3> coordinatesToWind(float latStart, float lngStart, float latEnd, float lngEnd, float timeSeconds){
    float deltaLat = latEnd-latStart;
    float deltaLng = lngEnd - lngStart;

    float avgLatRadians = (latStart+latEnd) * 0.5 * (M_PI / 180.0);

    float displacement_x_m = deltaLng * R * cos(avgLatRadians);
    float displacement_y_m = deltaLat * R;

    float displacement = sqrt(dx * dx + dy * dy);
    float velocity = displacement / timeSeconds;

    float direction = atan2(displacement_y_m,displacement_x_m) * 180 / M_PI;
    if (direction < 0) direction += 360.0;

    return { velocity, displacement, direction };

}