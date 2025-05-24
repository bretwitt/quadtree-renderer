#version 330 core

/*
   High‑precision vertex shader for planetary terrain with logarithmic depth.
   Matches the fragment shader "terrain_fragment_shader.glsl".
*/

// Vertex attributes ----------------------------------------------------------
layout(location = 0) in vec3 aPos;          // fine‑mesh position
layout(location = 1) in vec3 aNormal;       // fine‑mesh normal
layout(location = 2) in vec2 aTexCoords;    // UV
layout(location = 3) in vec3 aCoarse;       // coarser LOD position (for morphing)
layout(location = 4) in vec3 aCoarseNormal; // coarser LOD normal

// Uniforms -------------------------------------------------------------------
uniform mat4  model;
uniform mat4  view;
uniform mat4  projection;
uniform float splitTicks;   // morph time parameter  [0..1]

uniform vec3  cameraPos;    // in world space (double in C++, float here)
uniform float level;        // quadtree LOD level (0 = coarsest)

// Log‑depth constant:  C = 2 / log2(farPlane + 1) -----------------------------
uniform float logBufC;

// Outputs to fragment shader --------------------------------------------------
out vec3  FragPos;
out vec3  Normal;
out vec2  TexCoords;
out float vFragW;           // ORIGINAL clip‑space w for log‑depth

void main()
{
    // ----------------------------------------------------------------------
    // 1. LOD morphing (optional ‑‑ currently disabled / experimental)
    // ----------------------------------------------------------------------
    // Distance‑based morph factor (could also use splitTicks)
    const float thresholds[6] = float[](1200.0 + 1000.0,
                                        50.0  + 30.0,
                                        10.0  + 20.0,
                                        2.0   + 1.0,
                                        1.0   + 0.5,
                                        0.2   + 0.25);

    int clampedLevel  = int(clamp(level - 1.0, 0.0, 3.0));
    float maxDist     = thresholds[clampedLevel];
    float dist        = length(cameraPos - (model * vec4(aPos, 1.0)).xyz);
    float distanceFac = clamp(dist / maxDist, 0.0, 1.0);

    // Morph factor can be product of splitTicks and distanceFac
    float morphFac    = 0.0; // distanceFac * splitTicks; // ← enable if desired

    // Interpolate positions & normals
    vec3 morphPos     = mix(aPos,        aCoarse,        morphFac);
    vec3 morphNormal  = mix(aNormal,     aCoarseNormal,  morphFac);

    // ----------------------------------------------------------------------
    // 2. Transform to world space
    // ----------------------------------------------------------------------
    vec4 worldPos     = model * vec4(morphPos, 1.0);
    FragPos           = worldPos.xyz;
    Normal            = normalize(mat3(transpose(inverse(model))) * morphNormal);
    TexCoords         = aTexCoords;

    // ----------------------------------------------------------------------
    // 3. Camera / projection to clip space
    // ----------------------------------------------------------------------
    vec4 clipPos      = projection * view * worldPos;
    vFragW            = clipPos.w;                // keep original w for log‑depth

    // Logarithmic depth remap (Outerra formula)
    float logz        = log2(max(1e-6, clipPos.w + 1.0));
    clipPos.z         = logz * logBufC - 1.0;     // map to NDC [-1, +1]

    gl_Position       = clipPos;
}
