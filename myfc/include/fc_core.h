#ifndef CORE_H
#define CORE_H

#define DEG_PER_RAD 57.29577951308232f
#define RAD_PER_DEG 0.017453292519943295f
#define RE 6378137.0 // WGS84 equatorial radius, metres //for Pkf

#ifndef M_PI
#define M_PI		3.14159265358979323846
#endif

typedef struct
{
    float x;
    float y;
    float z;
} Vector3f;

typedef struct
{
    float w;
    float x;
    float y;
    float z;
} Quaternionf;

#endif