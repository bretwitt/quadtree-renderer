#include "QuadtreeRenderer.h"

QuadtreeRenderer::QuadtreeRenderer()
    : VAO(0), VBO(0)
{
    // Create OpenGL buffers.
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);
}

QuadtreeRenderer::~QuadtreeRenderer()
{
    glDeleteBuffers(1, &VBO);
    glDeleteVertexArrays(1, &VAO);
}

void QuadtreeRenderer::update(const QuadTree<int>* root)
{
    vertices.clear();
    if (root)
        collectQuads(root);

    // Bind and update buffer data.
    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);
    glBufferData(GL_ARRAY_BUFFER, vertices.size() * sizeof(float), vertices.data(), GL_DYNAMIC_DRAW);

    // Assuming our shader expects a vec3 position attribute at location 0.
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(0);

    // Unbind (optional)
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
}

void QuadtreeRenderer::draw() const
{
    glBindVertexArray(VAO);
    // Each rectangle is drawn as 4 independent line segments.
    // Since we push 8 vertices (2 per edge) per rectangle, the total number of vertices is vertices.size()/3.
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(vertices.size() / 3));
    glBindVertexArray(0);
}

void QuadtreeRenderer::collectQuads(const QuadTree<int>* node)
{
    if (!node)
        return;

    // Retrieve the boundary for the current node.
    auto boundary = node->getBoundary();
    float cx = boundary.x;
    float cy = boundary.y;
    float hw = boundary.width;
    float hh = boundary.height;

    // Compute the four corners of the rectangle.
    // (Note: The original quadtree uses (x, y) as the center, and (width, height) as half-dimensions.)
    float left   = cx - hw;
    float right  = cx + hw;
    float top    = cy - hh;
    float bottom = cy + hh;

    // For rendering in 3D, we use (x, y, z) coordinates.
    // Here we map our quadtree's (x,y) to (x,y) and use a fixed z coordinate.
    // Add line segments for the four edges (each edge as 2 vertices):

    // Top edge.
    addVertex(left, top, RENDER_Z);
    addVertex(right, top, RENDER_Z);
    
    // Right edge.
    addVertex(right, top, RENDER_Z);
    addVertex(right, bottom, RENDER_Z);
    
    // Bottom edge.
    addVertex(right, bottom, RENDER_Z);
    addVertex(left, bottom, RENDER_Z);
    
    // Left edge.
    addVertex(left, bottom, RENDER_Z);
    addVertex(left, top, RENDER_Z);

    // If the node has been subdivided, recursively collect children.
    if (node->isDivided()) {
        collectQuads(node->getNortheast());
        collectQuads(node->getNorthwest());
        collectQuads(node->getSoutheast());
        collectQuads(node->getSouthwest());
    }
}

void QuadtreeRenderer::addVertex(float x, float y, float z)
{
    vertices.push_back(x);
    vertices.push_back(y);
    vertices.push_back(z);
}