#version 330 core
in float height;
out vec4 FragColor;

void main()
{
    // Simple coloring based on height
    //vec3 color = vec3(0.0, 0.5 + height / 10.0, 0.0); // Adjusted to normalize height
    vec3 color = vec3(0.0, 0.0,1.0); // Adjusted to normalize height
    FragColor = vec4(color, 1.0);
}
