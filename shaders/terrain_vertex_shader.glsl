#version 330 core

// Vertex attributes
layout (location = 0) in vec3 aPos;       // Vertex position
layout (location = 1) in vec3 aNormal;    // Vertex normal
layout (location = 2) in vec2 aTexCoords; // Texture coordinates (optional)
layout (location = 3) in vec3 aCoarse; // Texture coordinates (optional)
layout (location = 4) in vec3 aCoarseNormal; // Texture coordinates (optional)

// Uniform matrices for transforming positions and normals
uniform mat4 model;
uniform mat4 view;
uniform mat4 projection;

// A morph factor [0..1]; 0 = original height, 1 = fully interpolated height
uniform float splitTicks;

uniform vec3 cameraPos;
uniform float level;

// Outputs to the fragment shader
out vec3 FragPos;
out vec3 Normal;
out vec2 TexCoords;


void main()
{
    vec4 worldPosD = model*vec4(aPos,1.0);
    float distance = length(cameraPos-worldPosD.xyz);

    float thresholds[6] = float[](1200.0 + 1000., 50.0 + 30.0, 10.0 + 20.0, 2.0 + 1.0,1.0 + 0.5,0.2 + 0.25);

    int clampedLevel = int(clamp(level - 1, 0.0, 3.0));
    float maxDist = thresholds[clampedLevel];
    //float maxDist = 1.0;


    float distanceFactor = clamp(distance / maxDist, 0.0, 1.0);

    float morphFac = distanceFactor; //*splitTicks;

    vec3 morphPos = mix(aPos, aCoarse, morphFac);

    vec4 worldPos = model * vec4(morphPos, 1.0);
    FragPos = worldPos.xyz;

    vec3 morphedNormal = mix(aNormal, aCoarseNormal, morphFac);


    Normal = normalize(mat3(transpose(inverse(model))) * morphedNormal);

    TexCoords = aTexCoords;
    gl_Position = projection * view * worldPos;
}
