cmake_minimum_required(VERSION 3.3)
project(HeightfieldRenderer)

# Set the C++ standard
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)


# Specify the sources
set(SOURCES
    src/main.cpp
    src/Application.cpp
    src/Shader.cpp
    src/Heightfield.cpp
    src/Camera.cpp
    src/GeoTIFFLoader.cpp
    src/QuadtreeRenderer.cpp
)

# Include directories
include_directories(
    include
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends)

file(GLOB IMGUI_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.h
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3.h
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3_loader.cpp
)

add_library("ImGui" STATIC ${IMGUI_SOURCES})

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Find the GDAL package.
# This will look for the GDALConfig.cmake file or use the FindGDAL module.
find_package(GDAL REQUIRED)

if(NOT GDAL_FOUND)
    message(FATAL_ERROR "GDAL not found. Please install GDAL and try again.")
endif()

message(STATUS "Found GDAL version: ${GDAL_VERSION_STRING}")
message(STATUS "GDAL include dir: ${GDAL_INCLUDE_DIRS}")
message(STATUS "GDAL libraries: ${GDAL_LIBRARIES}")


include_directories(${GDAL_INCLUDE_DIRS})

# Find OpenGL
find_package(OpenGL REQUIRED)
if (OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} ${OPENGL_gl_LIBRARY})
else()
    message(ERROR " OPENGL not found!")
endif()

# Find GLFW
# It's recommended to use the package provided by GLFW if installed system-wide
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

if (GLFW_FOUND)
    include_directories(${GLFW_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} ${GLFW_LIBRARIES})
else()
    message(ERROR " GLFW not found via pkg-config!")
endif()

# Find GLAD
# If GLAD is included as part of your project, you can add it as a subdirectory
# Alternatively, you can use a package manager or install it system-wide
# Here, we'll assume you have GLAD as part of your project

# Add GLAD
add_library(glad src/glad.c)  # Adjust the path if necessary
target_include_directories(glad PUBLIC include)  # Adjust include path if necessary
target_link_libraries(${PROJECT_NAME} glad ImGui ${GDAL_LIBRARIES})

# Link other necessary libraries
if(UNIX)
    target_link_libraries(${PROJECT_NAME} dl pthread X11)
endif()

# Optionally, link GLM if it's used (GLM is header-only)
# find_package(glm REQUIRED)
# if(glm_FOUND)
#     include_directories(${GLM_INCLUDE_DIRS})
# endif()

# Additional configurations (optional)
# You can set properties, compiler flags, etc., here
