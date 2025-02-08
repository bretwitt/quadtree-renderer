#version 330 core
out vec4 FragColor;

// From vertex shader
in vec3 FragPos;
in vec3 Normal;
in vec2 TexCoords;

// Lighting uniforms
uniform vec3 lightDir;    // Direction of the light (adjust sign as needed)
uniform vec3 lightColor;  // Color/intensity of the light
uniform vec3 viewPos;     // Camera position for specular

// Terrain texture (diffuse map)
uniform sampler2D terrainTexture;

void main()
{
    //----------------------------------------------------------
    // 1. Basic Ambient + Diffuse + Specular Lighting
    //----------------------------------------------------------
    // Ambient
    float ambientStrength = 0.8;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, -lightDir), 0.0);  
    vec3 diffuse = diff * lightColor;

    // Specular (simple Phong)
    float specularStrength = 0.5;
    float shininess = 32.0;
    vec3 viewDir = normalize(viewPos - FragPos);
    vec3 reflectDir = reflect(lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), shininess);
    vec3 specular = specularStrength * spec * lightColor;

    //----------------------------------------------------------
    // 2. Sample the Terrain Texture
    //----------------------------------------------------------
    // Use the TexCoords from the vertex shader
    vec4 texColor = texture(terrainTexture, TexCoords);

    //----------------------------------------------------------
    // 3. Combine Lighting with Texture
    //----------------------------------------------------------
    vec3 lighting = ambient + diffuse + specular;
    vec3 finalColor = texColor.rgb * lighting;  // modulate texture color by light

    // Output final fragment color
    FragColor = vec4(finalColor, texColor.a);
}
