#version 330 core
// Vertex attributes
layout (location = 0) in vec3 aPos;       // Vertex position
layout (location = 1) in vec3 aNormal;    // Vertex normal
layout (location = 2) in vec2 aTexCoords; // Texture coordinates (optional)

// Uniform matrices for transforming positions and normals
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Outputs to the fragment shader
out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

void main()
{
    // Transform the vertex position to world space
    vec4 worldPos = model * vec4(aPos, 1.0);
    FragPos = worldPos.xyz;

    // Transform the normal with the inverse transpose of the model matrix
    Normal = mat3(transpose(inverse(model))) * aNormal;

    // Pass along the texture coordinates
    TexCoords = aTexCoords;

    // Calculate the final vertex position in clip space
    gl_Position = projection * view * worldPos;
}
