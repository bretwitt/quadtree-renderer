// ─── GeoTIFFLoader.cpp (updated, safe deleter) ─────────────────────────────
#include "qtplanet/GeoTIFFLoader.h"

#include "gdal_priv.h"
#include "cpl_conv.h"   // CPLMalloc

#include <array>
#include <algorithm>
#include <cstring>
#include <iomanip>
#include <iostream>
#include <limits>
#include <memory>

// ────────────────────────────────────────────────────────────────────────────
GeoTIFFLoader::GeoTIFFLoader()
    : width(0), height(0), memoryUsage(0)
{
    GDALAllRegister();      // make sure every driver is available
}

GeoTIFFLoader::~GeoTIFFLoader() = default;

// --------------------------------------------------------------------------
void GeoTIFFLoader::load(const std::string &filename)
{
    /* 1.  Open dataset ----------------------------------------------------*/
    GDALDataset *poDataset = static_cast<GDALDataset *>(
        GDALOpen(filename.c_str(), GA_ReadOnly));
    if (!poDataset)
        throw std::runtime_error("Failed to open GeoTIFF file: " + filename);

    /* 2.  Raster geometry -------------------------------------------------*/
    width  = poDataset->GetRasterXSize();
    height = poDataset->GetRasterYSize();

    geoTransform.resize(6);
    if (poDataset->GetGeoTransform(geoTransform.data()) != CE_None) {
        GDALClose(poDataset);
        throw std::runtime_error("Failed to get geotransform from GeoTIFF file: " + filename);
    }

    /* 3.  Band 1 → elevation buffer (64‑bit) ------------------------------*/
    GDALRasterBand *poBand = poDataset->GetRasterBand(1);
    if (!poBand) {
        GDALClose(poDataset);
        throw std::runtime_error("Failed to get raster band from GeoTIFF file: " + filename);
    }

    std::size_t cells = static_cast<std::size_t>(width) * height;
    elevationData.resize(cells);

    memoryUsage = static_cast<int>(cells * sizeof(double) +
                                   geoTransform.size() * sizeof(double));

    CPLErr err = poBand->RasterIO(GF_Read,
                                  0, 0, width, height,
                                  elevationData.data(),
                                  width, height,
                                  GDT_Float64,   // tell GDAL we expect 64‑bit
                                  0, 0);
    if (err != CE_None) {
        GDALClose(poDataset);
        throw std::runtime_error("RasterIO failed for GeoTIFF file: " + filename);
    }

    /* 4.  Quick info to stderr (optional) ---------------------------------*/
    auto printLonLatSpan = [&](GDALDataset *ds,
                               const std::vector<double> &gt,
                               int w, int h)
    {
        /* 4.1  Source SRS --------------------------------------------------*/
        OGRSpatialReference srcSRS = getDatasetSRS(ds);
        srcSRS.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        /* 4.2  Target SRS: geographic version of source -------------------*/
        std::unique_ptr<OGRSpatialReference> dstSRS(
            srcSRS.IsGeographic() ? srcSRS.Clone() : srcSRS.CloneGeogCS());
        dstSRS->SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);

        /* Custom deleter so we can use unique_ptr safely */
        struct OCTDestroyer {
            void operator()(OGRCoordinateTransformation *p) const {
                if (p) OCTDestroyCoordinateTransformation(p);
            }
        };
        using OCTPtr = std::unique_ptr<OGRCoordinateTransformation, OCTDestroyer>;

        OCTPtr toGeo;  // nullptr by default, safe deleter attached

        if (!srcSRS.IsSame(dstSRS.get())) {
            toGeo.reset(OGRCreateCoordinateTransformation(&srcSRS, dstSRS.get()));
            if (!toGeo)
                throw std::runtime_error("PROJ can’t build the projected→geographic transform");
        }

        /* 4.3  Raster corner coordinates ----------------------------------*/
        std::array<double, 4> px = {0.0, double(w - 1), 0.0, double(w - 1)};
        std::array<double, 4> py = {0.0, 0.0, double(h - 1), double(h - 1)};
        std::array<double, 4> lon, lat;

        for (std::size_t i = 0; i < 4; ++i) {
            lon[i] = gt[0] + px[i] * gt[1] + py[i] * gt[2];
            lat[i] = gt[3] + px[i] * gt[4] + py[i] * gt[5];
            if (toGeo) toGeo->Transform(1, &lon[i], &lat[i]);
        }

        double minLon = *std::min_element(lon.begin(), lon.end());
        double maxLon = *std::max_element(lon.begin(), lon.end());
        double minLat = *std::min_element(lat.begin(), lat.end());
        double maxLat = *std::max_element(lat.begin(), lat.end());

        std::cout.setf(std::ios::fixed);
        std::cout << "\n─ Lon/Lat bounding box ────────────────────────────────\n"
                  << "  Min (lon,lat): (" << minLon << ", " << minLat << ")\n"
                  << "  Max (lon,lat): (" << maxLon << ", " << maxLat << ")\n"
                  << "  Lon span     : "  << (maxLon - minLon) << "°\n"
                  << "  Lat span     : "  << (maxLat - minLat) << "°\n"
                  << "────────────────────────────────────────────────────\n";

        std::cout << "─ Geo-transform ────────────────────────────────\n"
                  << "  Origin (lon,lat): (" << gt[0] << ", " << gt[3] << ")\n"
                  << "  Pixel size      : (" << gt[1] << ", " << gt[5] << ")\n"
                  << "  Rotation        : (" << gt[2] << ", " << gt[4] << ")\n"
                  << "────────────────────────────────────────────────────\n";
    };

    printLonLatSpan(poDataset, geoTransform, width, height);

    /* 5.  Close dataset ---------------------------------------------------*/
    GDALClose(poDataset);
}

// --------------------------------------------------------------------------
OGRSpatialReference GeoTIFFLoader::getDatasetSRS(GDALDataset *ds)
{
    OGRSpatialReference srs;

    /* 1.  Prefer the WKT string (works on every GDAL version) */
    if (const char *wkt = ds->GetProjectionRef();
        wkt && std::strlen(wkt) > 0)
    {
        if (srs.SetFromUserInput(wkt) == OGRERR_NONE &&
            srs.Validate()          == OGRERR_NONE)
        {
            return srs;
        }
        std::cerr << "[GeoTIFFLoader] WKT present but could not be parsed; "
                     "falling back to GetSpatialRef().\n";
    }

#if GDAL_VERSION_MAJOR >= 3
    /* 2.  GDAL ≥ 3: clone the live SRS object */
    if (const OGRSpatialReference *live = ds->GetSpatialRef()) {
        std::unique_ptr<OGRSpatialReference> copy(live->Clone());
        if (copy && copy->Validate() == OGRERR_NONE)
            return *copy;
    }
#endif

    /* 3.  No valid CRS → assume WGS 84 */
    std::cerr << "[GeoTIFFLoader] Dataset has no valid CRS; assuming EPSG:4326\n";
    srs.SetWellKnownGeogCS("WGS84");
    return srs;
}

// --------------------------------------------------------------------------
int  GeoTIFFLoader::getWidth()        const { return width;  }
int  GeoTIFFLoader::getHeight()       const { return height; }
int  GeoTIFFLoader::getMemoryUsage()  const { return memoryUsage; }
const std::vector<double>& GeoTIFFLoader::getElevationData() const { return elevationData; }
const std::vector<double>& GeoTIFFLoader::getGeoTransform()  const { return geoTransform; }
