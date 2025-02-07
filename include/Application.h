// Application.h
#ifndef APPLICATION_H
#define APPLICATION_H

#include <glad/glad.h>

// #include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "Shader.h"
#include "Heightfield.h"
#include "Camera.h"
#include "QuadtreeTile.h"
#include "QuadtreeWorld.h"

#include "QuadtreeRenderer.h"
#include "BucketMeshRenderer.h"

class Application
{
public:
    // Constructor
    Application(int width, int height, const char* title);
    
    // Destructor
    ~Application();
    
    // Run the application
    void run();

private:
    // Window parameters
    int SCR_WIDTH;
    int SCR_HEIGHT;
    const char* windowTitle;
    GLFWwindow* window;

    // Shader and Heightfield
    Shader* shader;
    Heightfield* heightfield;
    // QuadtreeTile<int> qt;
    QuadtreeWorld qt_world;
    QuadtreeRenderer* renderer;
    BucketMeshRenderer* bucket_renderer;

    Shader* terrainShader;         // New terrain shader pointer
    bool terrainShaderActive;      // Whether the terrain shader is active
    bool terrainShaderKeyPressed;  // To prevent repeated toggling on key hold


    // Camera
    Camera camera;
    float lastX;
    float lastY;
    bool firstMouse;

    // Timing
    float deltaTime;
    float lastFrame;

    // Wireframe mode
    bool wireframe;
    bool wireframeKeyPressed;

    // Private methods
    bool init();
    void processInput();
    static void framebuffer_size_callback(GLFWwindow* window, int width, int height);
    void mouse_callback(double xpos, double ypos);
    void scroll_callback(double xoffset, double yoffset);

    // Static callbacks to interface with GLFW
    static void glfwFramebufferSizeCallback(GLFWwindow* window, int width, int height);
    static void glfwMouseCallback(GLFWwindow* window, double xpos, double ypos);
    static void glfwScrollCallback(GLFWwindow* window, double xoffset, double yoffset);
    
    // Helper to set user pointer for callbacks
    void setCallbacks();

    
};

#endif
