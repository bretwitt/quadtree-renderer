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

    static QuadTreeChildBounds<Cartesian> getChildBounds(const Cartesian::Boundary& b) {
        QuadTreeChildBounds<Cartesian> children;
        float hw = b.width / 2.0f;
        float hh = b.height / 2.0f;

        children.bounds[0] = { b.x + hw, b.y - hh, hw, hh }; // NE
        children.bounds[1] = { b.x - hw, b.y - hh, hw, hh }; // NW
        children.bounds[2] = { b.x + hw, b.y + hh, hw, hh }; // SE
        children.bounds[3] = { b.x - hw, b.y + hh, hw, hh }; // SW

        return children;
    }

    static bool contains(const Cartesian::Boundary& b, float x, float y) {
        float left = b.x - b.width;
        float right = b.x + b.width;
        float top = b.y - b.height;
        float bottom = b.y + b.height;
        return (x >= left && x <= right && y >= top && y <= bottom);
    }

    static float distanceTo(const Cartesian::Boundary& b, float x, float y, float z, float elevation) {
        float dx = std::max(std::abs(x - b.x) - b.width, 0.0f);
        float dy = std::max(std::abs(y - b.y) - b.height, 0.0f);
        float dz = elevation - z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    static std::array<Cartesian::Boundary, 4> subdivide(const Cartesian::Boundary& b) {
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

    static std::pair<float, float> cartesianAt(const Cartesian::Boundary& b, int i, int j, int divisions) {
        float stepX = (2.0f * b.width) / divisions;
        float stepY = (2.0f * b.height) / divisions;
        float startX = b.x - b.width;
        float startY = b.y - b.height;
        return { startX + i * stepX, startY + j * stepY };
    }
};

template<>
struct CoordinateTraits<Polar> {

    static QuadTreeChildBounds<Polar> getChildBounds(const Polar::Boundary& b) {
        QuadTreeChildBounds<Polar> children;
        float halfDR = b.dr / 2.0f;
        float halfDTheta = b.dtheta / 2.0f;

        children.bounds[0] = { b.r + halfDR, b.theta + halfDTheta, halfDR, halfDTheta }; // Outer-right
        children.bounds[1] = { b.r + halfDR, b.theta - halfDTheta, halfDR, halfDTheta }; // Outer-left
        children.bounds[2] = { b.r - halfDR, b.theta + halfDTheta, halfDR, halfDTheta }; // Inner-right
        children.bounds[3] = { b.r - halfDR, b.theta - halfDTheta, halfDR, halfDTheta }; // Inner-left

        return children;
    }

    static std::pair<float, float> cartesianAt(const Polar::Boundary& b, int i, int j, int divisions) {
        float dr = b.dr / divisions;
        float dTheta = b.dtheta / divisions;

        float r = b.r - b.dr / 2 + j * dr;
        float theta = b.theta - b.dtheta / 2 + i * dTheta;

        float x = r * std::cos(theta);
        float y = r * std::sin(theta);
        return { x, y };
    }
};

#endif // COORDINATE_SYSTEMS_H
