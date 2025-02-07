#version 330 core
out vec4 FragColor;

// Inputs from the vertex shader
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

// Uniforms for lighting and view information
uniform vec3 lightDir;       // Directional light direction (should be normalized)
uniform vec3 lightColor;     // Color of the light
uniform vec3 viewPos;        // Camera position

void main()
{
    // ------------------------------------------------
    // 1. Ambient Lighting
    // ------------------------------------------------
    float ambientStrength = 0.8;
    vec3 ambient = ambientStrength * lightColor;

    // ------------------------------------------------
    // 2. Diffuse Lighting
    // ------------------------------------------------
    vec3 norm = normalize(Normal);
    // lightDir is assumed to be pointing TOWARD the surface from the light.
    // If your lightDir is the direction FROM the surface to the light, remove the negative sign.
    float diff = max(dot(norm, -lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // ------------------------------------------------
    // 3. Specular Lighting (for extra detail)
    // ------------------------------------------------
    // Basic Blinn-Phong or Phong approach
    // Here we use reflection vector for Phong shading
    float specularStrength = 0.5;   // Adjust as desired
    float shininess = 32.0;        // Adjust as desired
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
    vec3 specular = specularStrength * spec * lightColor;

    // ------------------------------------------------
    // 4. Multi-Step Terrain Coloring Based on Height
    // ------------------------------------------------
    // We'll blend from a lowColor to a midColor, then from midColor to highColor,
    // using a height factor in the range [0.0, 1.0].
    //
    // Adjust your altitude range (-1700 to -1300) as needed.
    // heightFactor = 0 when y = -1700, 
    // heightFactor = 1 when y = -1300.
    //float heightFactor = clamp((FragPos.y + 1700.0) / 400.0, 0.0, 1.0);

    // Define three colors for low, mid, and high altitudes
    //vec3 colorLow  = vec3(0.2, 0.6, 0.2); // greenish (e.g., grass)
    //vec3 colorMid  = vec3(0.4, 0.3, 0.2); // brownish (e.g., rock/dirt)
    vec3 colorHigh = vec3(0.8, 0.8, 0.8); // near white (e.g., snow)

    // ------------------------------------------------
    // 5. Combine Lighting with Terrain Color
    // ------------------------------------------------
    vec3 finalColor = (ambient + diffuse + specular) * colorHigh;

    FragColor = vec4(finalColor, 1.0);
}
