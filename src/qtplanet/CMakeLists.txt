find_package(GDAL REQUIRED)

if(NOT GDAL_FOUND)
    message(FATAL_ERROR "GDAL not found. Please install GDAL and try again.")
endif()

message(STATUS "Found GDAL version: ${GDAL_VERSION_STRING}")
message(STATUS "GDAL include dir: ${GDAL_INCLUDE_DIRS}")
message(STATUS "GDAL libraries: ${GDAL_LIBRARIES}")


include_directories(${GDAL_INCLUDE_DIRS})


file(GLOB QTPLANET_SRC
    "*.cpp"
)

add_library(QuadtreePlanetLib ${QTPLANET_SRC})

target_include_directories(QuadtreePlanetLib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
