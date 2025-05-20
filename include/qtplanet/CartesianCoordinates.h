#ifndef QTPLANET_CARTESIANCOORDINATES_H
#define QTPLANET_CARTESIANCOORDINATES_H

#include "CoordinateSystems.h"
#include <array>

// forward-declare your traits (definitions follow in .cpp)
template<>
struct CoordinateTraits<Cartesian> {
    static std::array<Cartesian::Boundary,4> getChildBounds(const Cartesian::Boundary&);
    static bool contains(const Cartesian::Boundary&, float x, float y);
    static float distanceTo(const Cartesian::Boundary&, float x, float y, float z, float elevation);
    static std::array<Cartesian::Boundary,4> subdivide(const Cartesian::Boundary&);
    static float distance(Cartesian::Position, Cartesian::Position);
    static std::pair<float,float> cartesianAt(const Cartesian::Boundary&, int i, int j, int divisions);
};

#endif // QTPLANET_CARTESIANCOORDINATES_H