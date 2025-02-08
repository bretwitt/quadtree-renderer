#version 330 core

// Vertex attributes
layout (location = 0) in vec3 aPos;       // Vertex position
layout (location = 1) in vec3 aNormal;    // Vertex normal
layout (location = 2) in vec2 aTexCoords; // Texture coordinates (optional)

// Uniform matrices for transforming positions and normals
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// Four corners of a plane, each with a Y "height" we want to interpolate
uniform vec3 uLVertex; // Upper-left corner
uniform vec3 uRVertex; // Upper-right corner
uniform vec3 bLVertex; // Bottom-left corner
uniform vec3 bRVertex; // Bottom-right corner

// A morph factor [0..1]; 0 = original height, 1 = fully interpolated height
uniform float splitTicks;

// Outputs to the fragment shader
out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;

void main()
{

    vec3 finalLocalPos = aPos;
    finalLocalPos.z = mix(aPos.z, uLVertex.z, 0.0);
    vec4 worldPos = model * vec4(finalLocalPos, 1.0);
    FragPos = worldPos.xyz;

    Normal = mat3(transpose(inverse(model))) * aNormal;
    TexCoords = aTexCoords;
    gl_Position = projection * view * worldPos;
}
