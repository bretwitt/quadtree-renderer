#version 330 core

layout(location = 0) out vec4 FragColor;

in vec3  FragPos;
in vec3  Normal;
in vec2  TexCoords;
in float vFragW;

// Lighting uniforms
uniform vec3 lightDir;     // Direction TO the light (‑lightDir is from the light)
uniform vec3 lightColor;   // Colour / intensity
uniform vec3 viewPos;      // Camera position for specular

// Texture
uniform sampler2D terrainTexture;

// --- NEW: constant sent from CPU each frame ------------------------
uniform float logBufC;     // C = 2 / log2(far + 1)

void main()
{
    // --------------------------------------------------------------
    // 1. Phong lighting (ambient + diffuse + specular)
    // --------------------------------------------------------------
    // Ambient
    const float ambientStrength = 0.5;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    vec3 norm  = normalize(Normal);
    float diff = max(dot(norm, -lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular
    const float specularStrength = 0.5;
    const float shininess        = 32.0;
    vec3 viewDir    = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(-lightDir, norm);       // note the minus sign
    float spec      = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
    vec3 specular   = specularStrength * spec * lightColor;

    // --------------------------------------------------------------
    // 2. Texture sample
    // --------------------------------------------------------------
    vec4 texColor = texture(terrainTexture, TexCoords);

    // --------------------------------------------------------------
    // 3. Combine
    // --------------------------------------------------------------
    vec3 lighting   = ambient + diffuse + specular;
    vec3 finalColor = texColor.rgb * lighting;
    FragColor       = vec4(finalColor, texColor.a);

    // --------------------------------------------------------------
    // 4. Logarithmic depth write (must match vertex shader)
    // --------------------------------------------------------------
    float logz      = log2(max(1e-6, vFragW + 1.0));
    float depthClip = logz * logBufC - 1.0;      // [-1,1] clip‑space depth
    gl_FragDepth    = depthClip * 0.5 + 0.5;     // [0,1] range buffer
}
