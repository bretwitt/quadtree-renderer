cmake_minimum_required(VERSION 3.6)
project(HeightfieldRenderer)

# Set the C++ standard
set(CMAKE_CXX_STANDARD     17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS    OFF)

# Specify the sources
set(SOURCES
    src/main.cpp
    src/Application.cpp
    src/Shader.cpp
    src/Heightfield.cpp
    src/TextureLoader.cpp
)

# Include directories
include_directories(
    include
    thirdparty/stb 
)

file(GLOB IMGUI_SOURCES 
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/*.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_glfw.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3.cpp
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends/imgui_impl_opengl3_loader.cpp
)

add_library(ImGui STATIC ${IMGUI_SOURCES})
set_target_properties(ImGui PROPERTIES
    LINKER_LANGUAGE CXX
    CXX_STANDARD 17
    CXX_STANDARD_REQUIRED ON
)
target_include_directories(ImGui PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui
    ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/imgui/backends
)

# Add executable
add_executable(${PROJECT_NAME} ${SOURCES})

# Link ImGui immediately after executable is declared
target_link_libraries(${PROJECT_NAME} PRIVATE ImGui)

# Add and link QuadtreePlanetLib
add_subdirectory(src/qtplanet)
target_link_libraries(${PROJECT_NAME} PRIVATE QuadtreePlanetLib)

# Find the GDAL package
find_package(GDAL REQUIRED)
if(NOT GDAL_FOUND)
    message(FATAL_ERROR "GDAL not found. Please install GDAL and try again.")
endif()
message(STATUS "Found GDAL version: ${GDAL_VERSION_STRING}")
message(STATUS "GDAL include dir: ${GDAL_INCLUDE_DIRS}")
message(STATUS "GDAL libraries: ${GDAL_LIBRARIES}")
include_directories(${GDAL_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME} PRIVATE ${GDAL_LIBRARIES})

# Find OpenGL
find_package(OpenGL REQUIRED)
if(OPENGL_FOUND)
    include_directories(${OPENGL_INCLUDE_DIR})
    target_link_libraries(${PROJECT_NAME} PRIVATE ${OPENGL_gl_LIBRARY})
else()
    message(FATAL_ERROR "OpenGL not found!")
endif()

# Find GLFW
find_package(glfw3 3.3 REQUIRED CONFIG)
# Link GLFW
target_link_libraries(${PROJECT_NAME} PRIVATE glfw)

# Add GLAD
add_library(glad src/glad.c)
target_include_directories(glad PUBLIC include)
target_link_libraries(${PROJECT_NAME} PRIVATE glad)

# On Linux platforms link extra libs
if(UNIX AND NOT APPLE)
    target_link_libraries(${PROJECT_NAME} PRIVATE dl pthread X11)
endif()

# Optionally link GLM (header-only)
# find_package(glm REQUIRED)
# if(glm_FOUND)
#     target_include_directories(${PROJECT_NAME} PRIVATE ${GLM_INCLUDE_DIRS})
# endif()
