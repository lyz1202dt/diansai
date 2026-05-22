#ifndef __DATAPACK_H__
#define __DATAPACK_H__

#include <cstdint>

#pragma pack(1)

typedef struct{
    float pos;
    float vel;
}Joint;

typedef struct{
    uint32_t head;
    Joint joint[3];
}TargetPackage;

typedef struct{
    uint32_t head;
    Joint joint[3];
}StatePackage;


#pragma pack()

#endif