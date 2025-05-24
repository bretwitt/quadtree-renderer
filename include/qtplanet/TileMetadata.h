
#ifndef QTPLANET_TILEMETADATA_H
#define QTPLANET_TILEMETADATA_H

#include <vector>

//---------------------------------------------------------------------
// A helper structure to uniquely identify a tile by its grid indices.
//---------------------------------------------------------------------
struct TileKey {
    int x;
    int y;
    
    bool operator==(const TileKey& other) const {
        return x == other.x && y == other.y;
    }
};

// Provide a hash function for TileKey so it can be used in an unordered_map.
namespace std {
    template <>
    struct hash<TileKey> {
        std::size_t operator()(const TileKey& key) const {
            std::size_t hx = std::hash<int>()(key.x);
            std::size_t hy = std::hash<int>()(key.y);
            return hx ^ (hy << 1);
        }
    };
}

struct vec3 {
    vec3(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }
    double x = 0;
    double y = 0;
    double z = 0;
};

struct TileMetadata {
    int ticksSinceSplit = 0; // -1 if expired/never happened
    int ticksSinceMerge = 0; // -1 if expired/never happened
    std::vector<vec3> dirtyVertices; // Deformations at highest LOD level
    bool dirtyVerticesTransferred = false;
};

#endif // QTPLANET_TILEMETADATA_H