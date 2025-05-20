#ifndef QTPLANET_COORDINATESYSTEMS_H
#define QTPLANET_COORDINATESYSTEMS_H


struct Cartesian {
    struct Position { float x, y; };
    struct Boundary { float x, y, width, height; };
};

struct Polar {
    struct Position { float lon, lat; };
    struct Boundary { float r, theta, dr, dtheta; };
};


template<typename CoordSystem>
struct CoordinateTraits;        // primary template left undefined

// specializations are declared in the system-specific headers:
#include "CartesianCoordinates.h"
#include "PolarCoordinates.h"

#endif // QTPLANET_COORDINATESYSTEMS_H