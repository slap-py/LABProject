#ifndef WINDTABLES_H
#define WINDTABLES_H

struct WindLayer{
    float altitudeMin;
    float altitudeMax;
    float velocity;
    float direction;
};

void recordLayer (float altitudeMin,float altitudeMax, float velocity, float direction);
void predictLanding(float burstLat, float burstLng, float burstElevation, WindLayer windTable[],int tableSize)


#endif