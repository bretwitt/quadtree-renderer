
#ifndef QTPLANET_TILEMETADATA_H
#define QTPLANET_TILEMETADATA_H

#include <vector>

struct vec3 {
    vec3(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    float x = 0;
    float y = 0;
    float z = 0;
};

struct TileMetadata {
    int ticksSinceSplit = 0; // -1 if expired/never happened
    int ticksSinceMerge = 0; // -1 if expired/never happened
    std::vector<vec3> dirtyVertices; // Deformations at highest LOD level
    bool dirtyVerticesTransferred = false;
};

#endif // QTPLANET_TILEMETADATA_H