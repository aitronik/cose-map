#pragma once
#include <stdint.h>

struct clusterData{
    uint8_t semId;
    float conf;

    float centerX;
    float centerY;
    float centerZ;
    
    float minX;
    float minY;
    float minZ;
    
    float maxX;
    float maxY;
    float maxZ;

    uint8_t r;
    uint8_t g;
    uint8_t b;
};

struct voxelData{
    uint8_t semId;
    float conf;

    uint8_t r;
    uint8_t g;
    uint8_t b;

    double x;
    double y;
    double z;

    float volume;
    int objId;
};