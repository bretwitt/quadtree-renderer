// ─── GeoTIFFLoader.h ───────────────────────────────────────────────────────
#pragma once

#include <string>
#include <vector>

#include <gdal_priv.h>
#include <ogr_spatialref.h>

/**
 * @brief Lightweight loader for single‑band GeoTIFF elevation tiles.
 *
 * After calling load(filename) you can access:
 *   • width(), height()           – raster size in pixels
 *   • getGeoTransform()           – 6‑element affine transform (GDAL order)
 *   • getElevationData()          – interleaved row‑major buffer (double)
 *
 * The class always reads the first raster band and promotes its values to
 * 64‑bit floating‑point (GDT_Float64). No rescaling or nodata handling is
 * done here; callers are expected to inspect and post‑process the buffer.
 */
class GeoTIFFLoader
{
public:
    GeoTIFFLoader();
    ~GeoTIFFLoader();

    /** Load a GeoTIFF from disk, throwing std::runtime_error on failure. */
    void load(const std::string& filename);

    // ── Accessors ──────────────────────────────────────────────────────────
    [[nodiscard]] int getWidth()  const;
    [[nodiscard]] int getHeight() const;
    [[nodiscard]] int getMemoryUsage() const;  //!< rough heap bytes used

    [[nodiscard]] const std::vector<double>& getElevationData() const;
    [[nodiscard]] const std::vector<double>& getGeoTransform()  const;

    /**
     * @brief Fetch the dataset's CRS as an OGRSpatialReference.
     *
     * Handles the quirks of older/newer GDAL versions and returns a valid
     * geographic SRS (EPSG:4326) if none is present in the file.
     */
    static OGRSpatialReference getDatasetSRS(GDALDataset* ds);

private:
    int width;       //!< number of columns (pixels)
    int height;      //!< number of rows    (pixels)
    int memoryUsage; //!< bytes allocated by elevationData + geoTransform

    std::vector<double> elevationData; //!< row‑major (height×width)
    std::vector<double> geoTransform;  //!< GDAL affine, size 6
};
