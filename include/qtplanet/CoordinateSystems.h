#ifndef COORDINATE_SYSTEMS_H
#define COORDINATE_SYSTEMS_H 
#include <cmath>
#include <cstdlib>
#include <algorithm> 
#include <array>
/*
*   Define coord systems
*/
struct Cartesian {
    struct Position {
        float x,y;
    };
    struct Boundary {
        float x, y, width, height;
    };
};
struct Polar {
    struct Position {
        float lon, lat;
    };
    struct Boundary {
        float r, theta, dr, dtheta;
    };
};

/*
*/
template<typename CoordSystem>
struct QuadTreeChildBounds {
    using Boundary = typename CoordSystem::Boundary;
    Boundary bounds[4];
};

/*
*/
template<typename T>
struct CoordinateTraits;

/*
*   Template specialized impl of common coordinate system ops
*
*/
template<>
struct CoordinateTraits<Cartesian> {
    using Boundary = Cartesian::Boundary;
    using Position = Cartesian::Position;

    static QuadTreeChildBounds<Cartesian> getChildBounds(const Boundary& b) {
        QuadTreeChildBounds<Cartesian> children;
        float hw = b.width / 2.0f;
        float hh = b.height / 2.0f;

        children.bounds[0] = { b.x + hw, b.y - hh, hw, hh }; // NE
        children.bounds[1] = { b.x - hw, b.y - hh, hw, hh }; // NW
        children.bounds[2] = { b.x + hw, b.y + hh, hw, hh }; // SE
        children.bounds[3] = { b.x - hw, b.y + hh, hw, hh }; // SW

        return children;
    }

    static bool contains(const Boundary& b, float x, float y) {
        float left = b.x - b.width;
        float right = b.x + b.width;
        float top = b.y - b.height;
        float bottom = b.y + b.height;
        return (x >= left && x <= right && y >= top && y <= bottom);
    }

    static float distanceTo(const Boundary& b, float x, float y, float z, float elevation) {
        float dx = std::max(std::abs(x - b.x) - b.width, 0.0f);
        float dy = std::max(std::abs(y - b.y) - b.height, 0.0f);
        float dz = elevation - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static std::array<Boundary, 4> subdivide(const Boundary& b) {
        float hw = b.width / 2.0f;
        float hh = b.height / 2.0f;
        return {{
            { b.x + hw, b.y - hh, hw, hh }, // NE
            { b.x - hw, b.y - hh, hw, hh }, // NW
            { b.x + hw, b.y + hh, hw, hh }, // SE
            { b.x - hw, b.y + hh, hw, hh }  // SW
        }};
    }

    static float distance(Cartesian::Position pos1, Cartesian::Position pos2) {
        float dx = pos1.x - pos1.x;
        float dy = pos2.y - pos1.y;
        return std::sqrt(dx*dx + dy*dy);
    }
};

template<>
struct CoordinateTraits<Polar> {
    using Boundary = Polar::Boundary;

    static QuadTreeChildBounds<Polar> getChildBounds(const Boundary& b) {
        QuadTreeChildBounds<Polar> children;
        float halfDR = b.dr / 2.0f;
        float halfDTheta = b.dtheta / 2.0f;

        children.bounds[0] = { b.r + halfDR, b.theta + halfDTheta, halfDR, halfDTheta }; // Outer-right
        children.bounds[1] = { b.r + halfDR, b.theta - halfDTheta, halfDR, halfDTheta }; // Outer-left
        children.bounds[2] = { b.r - halfDR, b.theta + halfDTheta, halfDR, halfDTheta }; // Inner-right
        children.bounds[3] = { b.r - halfDR, b.theta - halfDTheta, halfDR, halfDTheta }; // Inner-left

        return children;
    }
};

#endif // COORDINATE_SYSTEMS_H
