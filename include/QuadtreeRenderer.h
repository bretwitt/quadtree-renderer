#ifndef QUADTREE_RENDERER_H
#define QUADTREE_RENDERER_H

// Include GLEW first to prevent inclusion order issues.
#include "Quadtree.h"  // Ensure your Quadtree.h now includes the public accessors.
#include <vector>
#include <iostream>
#include <glad/glad.h>

// This renderer assumes the quadtree is for 2D data. If you want to overlay the quadtree
// on top of a 3D scene (for example, drawing in the xz-plane) you can adjust the Y coordinate
// accordingly. In this example we draw in the XY plane at a fixed Z value.
static const float RENDER_Z = 0.1f;  // Z coordinate for drawing (for example, just above the ground)

class QuadtreeRenderer {
public:
    QuadtreeRenderer();

    ~QuadtreeRenderer();

    // Update the vertex buffer with the current quadtree boundaries.
    // This method traverses the quadtree (using the public accessor methods)
    // and collects vertices for drawing each node’s boundary as 4 line segments.
    void update(const QuadTree<int>* root,GLuint shaderProgram);

    // Render the quadtree boundaries. Use an appropriate shader before calling this.
    void draw() const;

private:
    GLuint VAO, VBO;
    std::vector<float> vertices; // Each vertex is 3 floats: (x, y, z)

    // Recursively traverse the quadtree and add each node's boundary as line segments.
    void collectQuads(const QuadTree<int>* node);

    // Helper to add a vertex to the vertices vector.
    void addVertex(float x, float y, float z);
};
#endif // QUADTREE_RENDERER_H
