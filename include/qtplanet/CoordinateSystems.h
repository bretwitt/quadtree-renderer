#ifndef QTPLANET_COORDINATESYSTEMS_H
#define QTPLANET_COORDINATESYSTEMS_H

#include <vector>

struct Cartesian {
    struct Position { 
        double x, y; 
        std::vector<double> asArray() const { 
            return { x, y };
        }
    };
    struct Boundary { double x, y, width, height; };
};

struct Polar {
    struct Position { double lon, lat; };
    struct Boundary { double r, theta, dr, dtheta; };
};

struct Spherical {
    struct Position { 
        double lon, lat; 
        std::vector<double> asArray() const { 
            return { lon, lat };
        }

    };

    struct Boundary { double x, y, width, height; };
};


template<typename CoordSystem>
struct CoordinateTraits;        // primary template left undefined

// specializations are declared in the system-specific headers:
#include "CartesianCoordinates.h"
#include "SphericalCoordinates.h"

#endif // QTPLANET_COORDINATESYSTEMS_H