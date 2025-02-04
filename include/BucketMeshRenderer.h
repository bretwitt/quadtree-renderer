#ifndef QUADTREE_BUCKET_RENDERER_H
#define QUADTREE_BUCKET_RENDERER_H

#include "QuadtreeTile.h" // Assuming this header defines QuadTree<int>
#include <vector>
#include <iostream>
#include <glad/glad.h>
#include <unordered_map>


// Structure to hold GPU-related objects for a mesh.
struct GPU_Mesh {
    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLsizei indexCount = 0;
};

class BucketMeshRenderer {
public:
    // Maps a unique key (here a pointer to a QuadTree<int> instance) to its GPU buffers.
    std::unordered_map<QuadTree<int>*, GPU_Mesh> gpuMeshMap;

    ~BucketMeshRenderer() {
        // Cleanup GPU resources when the renderer is destroyed.
        for (auto& pair : gpuMeshMap) {
            glDeleteVertexArrays(1, &pair.second.VAO);
            glDeleteBuffers(1, &pair.second.VBO);
            glDeleteBuffers(1, &pair.second.EBO);
        }
    }

    // Updates (or creates) the GPU buffers for a given mesh.
    void updateMesh(QuadTree<int>* key, const Mesh& mesh) {
        GPU_Mesh gpuMesh;
        // Check if the GPU buffers for this mesh already exist.
        auto it = gpuMeshMap.find(key);
        if (it == gpuMeshMap.end()) {
            // Create new GPU buffers.
            glGenVertexArrays(1, &gpuMesh.VAO);
            glGenBuffers(1, &gpuMesh.VBO);
            glGenBuffers(1, &gpuMesh.EBO);
        } else {
            gpuMesh = it->second;
        }

        // Bind the Vertex Array Object.
        glBindVertexArray(gpuMesh.VAO);

        // Upload vertex data.
        glBindBuffer(GL_ARRAY_BUFFER, gpuMesh.VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     mesh.vertices.size() * sizeof(float),
                     mesh.vertices.data(),
                     GL_STATIC_DRAW);

        // Upload index data.
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuMesh.EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     mesh.indices.size() * sizeof(unsigned int),
                     mesh.indices.data(),
                     GL_STATIC_DRAW);

        // Set vertex attribute pointers.
        // Assuming layout location 0 for position, with 3 floats per vertex.
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Store the index count.
        gpuMesh.indexCount = static_cast<GLsizei>(mesh.indices.size());

        // Unbind the VAO (optional, but good practice)
        glBindVertexArray(0);

        // Save the GPU_Mesh for this key.
        gpuMeshMap[key] = gpuMesh;
    }

    // Renders all meshes stored in the bucketMeshes map.
    void render(const std::unordered_map<QuadTree<int>*, Mesh>& bucketMeshes, GLuint shaderProgram) {
        glUseProgram(shaderProgram);
        GLint colorLoc = glGetUniformLocation(shaderProgram, "levelColor");

        // Iterate over each bucket's mesh.
        for (const auto& entry : bucketMeshes) {
            QuadTree<int>* key = entry.first;
            const Mesh& mesh = entry.second;

            // Update (or create) the GPU buffers for this mesh.
            updateMesh(key, mesh);
            int level = key->getLevel(); // Expected to be between 0 and 5.
            float r, g, b;
            switch (level) {
                case 0: r = 1.0f; g = 0.0f; b = 0.0f; break; // Red
                case 1: r = 0.0f; g = 1.0f; b = 0.0f; break; // Green
                case 2: r = 0.0f; g = 0.0f; b = 1.0f; break; // Blue
                case 3: r = 1.0f; g = 1.0f; b = 0.0f; break; // Yellow
                case 4: r = 1.0f; g = 0.0f; b = 1.0f; break; // Magenta
                case 5: r = 0.0f; g = 1.0f; b = 1.0f; break; // Cyan
                default: r = 1.0f; g = 1.0f; b = 1.0f; break; // White (fallback)
            }
            glUniform3f(colorLoc, r, g, b);
            // Retrieve the GPU buffers.
            const GPU_Mesh& gpuMesh = gpuMeshMap[key];

            // Bind the VAO and draw the mesh.
            glBindVertexArray(gpuMesh.VAO);
            glDrawElements(GL_TRIANGLES, gpuMesh.indexCount, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
};

#endif // QUADTREE_BUCKET_RENDERER_H
