#ifndef QUADTREE_BUCKET_RENDERER_H
#define QUADTREE_BUCKET_RENDERER_H

#include "QuadtreeTile.h" // Assuming this header defines QuadTree<int>
#include <vector>
#include <iostream>
#include <glad/glad.h>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

// Structure to hold GPU-related objects for a mesh.
struct GPU_Mesh {
    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLsizei indexCount = 0;
};

class BucketMeshRenderer {
public:
    std::unordered_map<QuadTree<int>*, GPU_Mesh> gpuMeshMap;

    ~BucketMeshRenderer() {
        for (auto& pair : gpuMeshMap) {
            glDeleteVertexArrays(1, &pair.second.VAO);
            glDeleteBuffers(1, &pair.second.VBO);
            glDeleteBuffers(1, &pair.second.EBO);
        }
    }
    float computeTriangleArea(const glm::vec3& v0, const glm::vec3& v1, const glm::vec3& v2) {
        return 0.5f * glm::length(glm::cross(v1 - v0, v2 - v0));
    }

    void updateMesh(QuadTree<int>* key, const Mesh& mesh) {
        GPU_Mesh gpuMesh;
        auto it = gpuMeshMap.find(key);
        if (it == gpuMeshMap.end()) {
            glGenVertexArrays(1, &gpuMesh.VAO);
            glGenBuffers(1, &gpuMesh.VBO);
            glGenBuffers(1, &gpuMesh.EBO);
        } else {
            gpuMesh = it->second;
        }

        // Calculate normals on the fly:
        size_t vertexCount = mesh.vertices.size() / 3;
        std::vector<glm::vec3> positions(vertexCount);
        std::vector<glm::vec3> normals(vertexCount, glm::vec3(0.0f));

        for (size_t i = 0; i < vertexCount; i++) {
            positions[i] = glm::vec3(
                mesh.vertices[i * 3 + 0],
                mesh.vertices[i * 3 + 1],
                mesh.vertices[i * 3 + 2]
            );
            normals[i] = glm::vec3(
                mesh.normals[i*3+0],
                mesh.normals[i*3+1],
                mesh.normals[i*3+2]  
            );
        }

        // for (size_t i = 0; i < mesh.indices.size(); i += 3) {
        //     unsigned int i0 = mesh.indices[i];
        //     unsigned int i1 = mesh.indices[i + 1];
        //     unsigned int i2 = mesh.indices[i + 2];

        //     glm::vec3 v0 = positions[i0];
        //     glm::vec3 v1 = positions[i1];
        //     glm::vec3 v2 = positions[i2];

        //     glm::vec3 edge1 = v1 - v0;
        //     glm::vec3 edge2 = v2 - v0;
        //     glm::vec3 faceNormal = glm::cross(edge1, edge2);
        //     float area = glm::length(faceNormal);
        //     if (area < 1e-6f) continue; // Skip degenerate triangle
        //     faceNormal /= area;  

        //     normals[i0] += faceNormal * area;
        //     normals[i1] += faceNormal * area;
        //     normals[i2] += faceNormal * area;
        // }


        // for (size_t i = 0; i < normals.size(); i++) {
        //     normals[i] = glm::normalize(normals[i]);
        // }

        // Interleave positions and normals.
        // Each vertex will have 6 floats: [pos.x, pos.y, pos.z, norm.x, norm.y, norm.z]
        std::vector<float> interleavedData;
        interleavedData.reserve(vertexCount * 6);
        for (size_t i = 0; i < vertexCount; i++) {
            // Position:
            interleavedData.push_back(positions[i].x);
            interleavedData.push_back(positions[i].y);
            interleavedData.push_back(positions[i].z);
            // Normal:
            interleavedData.push_back(normals[i].x);
            interleavedData.push_back(normals[i].y);
            interleavedData.push_back(normals[i].z);
        }

        // Bind the Vertex Array Object.
        glBindVertexArray(gpuMesh.VAO);

        // Upload interleaved vertex data.
        glBindBuffer(GL_ARRAY_BUFFER, gpuMesh.VBO);
        glBufferData(GL_ARRAY_BUFFER,
                     interleavedData.size() * sizeof(float),
                     interleavedData.data(),
                     GL_STATIC_DRAW);

        // Upload index data.
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuMesh.EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                     mesh.indices.size() * sizeof(unsigned int),
                     mesh.indices.data(),
                     GL_STATIC_DRAW);

        // Setup vertex attribute pointers:
        // Layout location 0: position (first 3 floats)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Layout location 1: normal (next 3 floats)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        // Store the index count.
        gpuMesh.indexCount = static_cast<GLsizei>(mesh.indices.size());

        // Unbind the VAO.
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
