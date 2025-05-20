#ifndef QTPLANET_COORDINATESYSTEMS_H
#define QTPLANET_COORDINATESYSTEMS_H

#include <vector>

struct Cartesian {
    struct Position { 
        float x, y; 
        std::vector<float> asArray() const { 
            return { x, y };
        }
    };
    struct Boundary { float x, y, width, height; };
};

struct Polar {
    struct Position { float lon, lat; };
    struct Boundary { float r, theta, dr, dtheta; };
};

struct Spherical {
    struct Position { 
        float lon, lat; 
        std::vector<float> asArray() const { 
            return { lon, lat };
        }

    };

    struct Boundary { float x, y, width, height; };
};


template<typename CoordSystem>
struct CoordinateTraits;        // primary template left undefined

// specializations are declared in the system-specific headers:
#include "CartesianCoordinates.h"
#include "SphericalCoordinates.h"

#endif // QTPLANET_COORDINATESYSTEMS_H