#ifndef QTPLANET_CARTESIANCOORDINATES_H
#define QTPLANET_CARTESIANCOORDINATES_H

#include <array>
#include <cmath>
#include <tuple>
#include "CoordinateSystems.h"
#include "TileMetadata.h"
#include "Perlin.h"

// forward declares
template<typename T, typename CoordSystem> class QuadTree;
class GeoTIFFLoader;
class MultiGeoTIFFManager;

template<>
struct CoordinateTraits<Cartesian> {
  using Boundary = Cartesian::Boundary;
  using Position = Cartesian::Position;

  static std::array<Boundary,4> getChildBounds(const Boundary& b);
  static bool contains(const Boundary& b, double x, double y);
  static double distanceToBounds(const Boundary& b,
                                double cx, double cy, double cz,
                                double elevation);
  static std::array<Boundary,4> subdivide(const Boundary& b);
  static double distance(Position p1, Position p2);

  static std::tuple<double,double,double>
    cartesianAt(const Boundary& b,
                int i, int j, int divisions,
                QuadTree<TileMetadata,Cartesian>* tree,
                const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
                int zoomLevel = 0);

  static double computeBaseElevation(const Position& pos,
                                    const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
                                    int zoomLevel = 0);

  static QuadTree<TileMetadata,Cartesian>*
    findLeafNode(QuadTree<TileMetadata,Cartesian>* node,
                 Position pos);

  static double getElevation(const Position& pos,
                            QuadTree<TileMetadata,Cartesian>* tree,
                            const std::shared_ptr<MultiGeoTIFFManager>& geoLoader,
                            int zoomLevel = 0);

  static std::pair<double,double> tileCenterPosition(const TileKey& key, double tileSize);
  static std::pair<int,int> computeTileIndices(const Position& pos, double tileSize);

};

#endif // QTPLANET_CARTESIANCOORDINATES_H
