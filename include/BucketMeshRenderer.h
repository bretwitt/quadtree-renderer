#ifndef QUADTREE_BUCKET_RENDERER_H
#define QUADTREE_BUCKET_RENDERER_H

#include "QuadtreeTile.h" // Assuming this header defines QuadTree<int>
#include <vector>
#include <iostream>
#include <glad/glad.h>
#include <unordered_map>
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "stb_image.h"

// Structure to hold GPU-related objects for a mesh.
struct GPU_Mesh {
    GLuint VAO = 0;
    GLuint VBO = 0;
    GLuint EBO = 0;
    GLsizei indexCount = 0;
};

class BucketMeshRenderer {
public:
    std::unordered_map<QuadTree<TileMetadata>*, GPU_Mesh> gpuMeshMap;

    GLuint terrainTextureID = 0;
    BucketMeshRenderer() {
        // 1. Generate a texture object and store its ID:
        glGenTextures(1, &terrainTextureID);

        // 2. Bind the texture so we can work with it:
        glBindTexture(GL_TEXTURE_2D, terrainTextureID);

        // 3. Upload or specify your texture data (width, height, format, etc.)
        //    For example, using stbi to load an image:
        int tWidth,tHeight,nrChannels;
        unsigned char* data = stbi_load("../resources/checkerboard.png", &tWidth, &tHeight, &nrChannels, 0);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, tWidth, tHeight, 0, GL_RGBA, GL_UNSIGNED_BYTE, data);

        // 4. Set texture parameters (wrapping, filtering, etc.):
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

        // 5. Generate mipmaps if desired:
        glGenerateMipmap(GL_TEXTURE_2D);

        // 6. Unbind texture if you want (optional):
        glBindTexture(GL_TEXTURE_2D, 0);

    }
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

    void updateMesh(QuadTree<TileMetadata>* key, const Mesh& mesh) {
        GPU_Mesh gpuMesh;
        auto it = gpuMeshMap.find(key);
        if (it == gpuMeshMap.end()) {
            glGenVertexArrays(1, &gpuMesh.VAO);
            glGenBuffers(1, &gpuMesh.VBO);
            glGenBuffers(1, &gpuMesh.EBO);
        } else {
            gpuMesh = it->second;
        }

        const size_t vertexCount = mesh.vertices.size() / 3;

        // -- Prepare to interleave data:
        //    (pos.x, pos.y, pos.z, norm.x, norm.y, norm.z, tex.u, tex.v)
        std::vector<float> interleavedData;
        interleavedData.reserve(vertexCount * 8); // 8 floats per vertex

        for (size_t i = 0; i < vertexCount; i++) {
            // Position (3 floats)
            float px = mesh.vertices[i * 3 + 0];
            float py = mesh.vertices[i * 3 + 1];
            float pz = mesh.vertices[i * 3 + 2];

            // Normal (3 floats)
            float nx = mesh.normals[i * 3 + 0];
            float ny = mesh.normals[i * 3 + 1];
            float nz = mesh.normals[i * 3 + 2];

            // TexCoord (2 floats)
            // Make sure mesh.texCoords has valid data for each vertex
            float u = mesh.texCoords[i * 2 + 0];
            float v = mesh.texCoords[i * 2 + 1];

            // Add to our interleaved buffer
            interleavedData.push_back(px);
            interleavedData.push_back(py);
            interleavedData.push_back(pz);
            interleavedData.push_back(nx);
            interleavedData.push_back(ny);
            interleavedData.push_back(nz);
            interleavedData.push_back(u);
            interleavedData.push_back(v);
        }

        // Bind & upload data to the GPU
        glBindVertexArray(gpuMesh.VAO);

        // VBO for our interleaved vertex data
        glBindBuffer(GL_ARRAY_BUFFER, gpuMesh.VBO);
        glBufferData(GL_ARRAY_BUFFER,
                    interleavedData.size() * sizeof(float),
                    interleavedData.data(),
                    GL_STATIC_DRAW);

        // EBO for indices
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, gpuMesh.EBO);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER,
                    mesh.indices.size() * sizeof(unsigned int),
                    mesh.indices.data(),
                    GL_STATIC_DRAW);

        // -----------------------------------------------------------------
        // Setup attribute pointers
        // 
        // Stride = 8 * sizeof(float)
        // Layout:
        //   0..2   -> Position
        //   3..5   -> Normal
        //   6..7   -> TexCoord
        // -----------------------------------------------------------------

        // Position -> layout (location = 0)
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
        glEnableVertexAttribArray(0);

        // Normal -> layout (location = 1)
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
        glEnableVertexAttribArray(1);

        // TexCoord -> layout (location = 2)
        glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
        glEnableVertexAttribArray(2);


        // Store index count
        gpuMesh.indexCount = static_cast<GLsizei>(mesh.indices.size());

        glBindVertexArray(0);

        // Update our map
        gpuMeshMap[key] = gpuMesh;
    }

    // Renders all meshes stored in the bucketMeshes map.
    void render(const std::unordered_map<QuadTree<TileMetadata>*, Mesh>& bucketMeshes, GLuint shaderProgram) {
        glUseProgram(shaderProgram);
        GLint colorLoc = glGetUniformLocation(shaderProgram, "levelColor");

        // Get uniform locations for the corner vertices (expected as vec3 in the shader)
        GLint uLVertexLoc = glGetUniformLocation(shaderProgram, "uLVertex");
        GLint uRVertexLoc = glGetUniformLocation(shaderProgram, "uRVertex");
        GLint bLVertexLoc = glGetUniformLocation(shaderProgram, "bLVertex");
        GLint bRVertexLoc = glGetUniformLocation(shaderProgram, "bRVertex");


        glActiveTexture(GL_TEXTURE0);
        glBindTexture(GL_TEXTURE_2D, terrainTextureID);

        // Let the shader know which texture unit to use (0).
        GLint textureLoc = glGetUniformLocation(shaderProgram, "terrainTexture");
        glUniform1i(textureLoc, 0);
        
        // Iterate over each bucket's mesh.
        for (const auto& entry : bucketMeshes) {
            QuadTree<TileMetadata>* key = entry.first;
            const Mesh& mesh = entry.second;

            // Update (or create) the GPU buffers for this mesh.
            updateMesh(key, mesh);

            // Choose a color based on the tile's level.
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

            auto boundary = key->getBoundary();

            // --- Calculate boundaries from center (x, y) and width ---
            // Assuming the QuadTree node (or its metadata) provides these functions:
            float centerX = boundary.x;     // Tile center x-coordinate
            float centerY = boundary.y;     // Tile center y-coordinate
            float tileWidth = boundary.width; // Tile width (assumed square)

            float halfWidth = tileWidth * 0.5f;
            float left   = centerX - halfWidth;
            float right  = centerX + halfWidth;
            float bottom = centerY - halfWidth;
            float top    = centerY + halfWidth;
            int gridWidth = 256 / (1 << key->getLevel());
            int gridHeight = 256 / (1 << key->getLevel());

            // --- Compute the height for each corner ---
            int indexUL = 0;                          // Upper-left: row 0, col 0
            int indexUR = gridWidth - 1;                // Upper-right: row 0, col (gridWidth - 1)
            int indexBL = (gridHeight - 1) * gridWidth; // Bottom-left: row (gridHeight - 1), col 0
            int indexBR = (gridHeight - 1) * gridWidth + (gridWidth - 1); // Bottom-right

            // Now, retrieve the height values from mesh.vertices:
            float heightUL = mesh.vertices[indexUL+2];
            float heightUR = mesh.vertices[(3*indexUR)+2];
            float heightBL = mesh.vertices[(3*indexBL)+2];
            float heightBR = mesh.vertices[(3*indexBR)+2];


            // Pass the corner positions (with heights) to the shader.
            // Here we assume the shader expects a vec3 with (x, height, y).
            glUniform3f(uLVertexLoc, left, top, heightUL);
            glUniform3f(uRVertexLoc, right, top, heightUR);
            glUniform3f(bLVertexLoc, left, bottom, heightBL);
            glUniform3f(bRVertexLoc, right,bottom, heightBR);

            std::cout << heightUL << std::endl;



            float splitTicks_ = key->getType()->ticksSinceSplit;

            if(splitTicks_ != -1) {
                splitTicks_ = 1. - fmin(splitTicks_/100.,1.0);
            } else {
                splitTicks_ = 0.0;
            }
            
            GLint splitTicksLoc = glGetUniformLocation(shaderProgram, "splitTicks");
            glUniform1f(splitTicksLoc, splitTicks_);

            // glUniform1f()
            // --- Render the mesh ---
            // Retrieve the GPU buffers (assuming gpuMeshMap is accessible)
            const GPU_Mesh& gpuMesh = gpuMeshMap[key];

            glBindVertexArray(gpuMesh.VAO);
            glDrawElements(GL_TRIANGLES, gpuMesh.indexCount, GL_UNSIGNED_INT, 0);
            glBindVertexArray(0);
        }
    }
};

#endif // QUADTREE_BUCKET_RENDERER_H
