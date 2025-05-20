// Application.cpp
#include "Application.h"
#include <iostream>

// Include ImGui headers
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"

const int DEFAULT_GRID_SIZE = 100;
const float DEFAULT_GRID_SCALE = 1.5f;
const float DEFAULT_HEIGHT_SCALE = 10.0f;


Application::Application(int width, int height, const char* title)
    : SCR_WIDTH(width), SCR_HEIGHT(height), windowTitle(title), window(nullptr),
      shader(nullptr), heightfield(nullptr), 
      camera(glm::vec3(-100.0f, -100.0f, -0.0f)), 
      lastX(width / 2.0f), lastY(height / 2.0f),
      firstMouse(true), deltaTime(0.0f), lastFrame(0.0f),
      wireframe(false), wireframeKeyPressed(false),
      qt_world(16, 10, 1200.0f,120.0f),
      bucket_renderer(nullptr), 
      terrainShader(nullptr),      
      terrainShaderActive(false),
      terrainShaderKeyPressed(false)
{
    if(!init())
    {
        std::cerr << "Failed to initialize Application\n";
        exit(EXIT_FAILURE);
    }
}

Application::~Application()
{
    // Cleanup ImGui
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext();

    // Cleanup application resources
    delete shader;
    delete heightfield;
    delete bucket_renderer;
    delete terrainShader;
    glfwTerminate();
}

bool Application::init()
{
    // Initialize GLFW
    if(!glfwInit())
    {
        std::cerr << "Failed to initialize GLFW\n";
        return false;
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);


    #ifdef __APPLE__
        glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    #endif

    // Create window
    window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, windowTitle, NULL, NULL);
    if(window == NULL)
    {
        std::cerr << "Failed to create GLFW window\n";
        glfwTerminate();
        return false;
    }
    glfwMakeContextCurrent(window);

    // Load OpenGL function pointers using GLAD
    if(!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
    {
        std::cerr << "Failed to initialize GLAD\n";
        return false;
    }

    #ifdef __APPLE__
        // On macOS Retina, window size (points) ≠ framebuffer size (pixels)
        int fbW, fbH;
        glfwGetFramebufferSize(window, &fbW, &fbH);
        glViewport(0, 0, fbW, fbH);
    #else    
        // On other platforms points == pixels
        glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);
    #endif

    glfwSetWindowUserPointer(window, this);
    setCallbacks();
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);
    glEnable(GL_DEPTH_TEST);

    // Initialize Shader
    shader = new Shader("../shaders/vertex_shader.glsl", "../shaders/fragment_shader.glsl");
    terrainShader = new Shader("../shaders/terrain_vertex_shader.glsl", "../shaders/terrain_fragment_shader.glsl");

    // heightfield = new Heightfield(DEFAULT_GRID_SIZE, DEFAULT_GRID_SCALE, DEFAULT_HEIGHT_SCALE);

    // Initialize renderers
    bucket_renderer = new BucketMeshRenderer();

    // ---------------------------
    // Initialize Dear ImGui
    // ---------------------------
    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO(); (void)io;
    ImGui::StyleColorsDark();

    const char* glsl_version = "#version 330";
    if (!ImGui_ImplGlfw_InitForOpenGL(window, true))
    {
        std::cerr << "Failed to initialize ImGui GLFW backend\n";
        return false;
    }
    if (!ImGui_ImplOpenGL3_Init(glsl_version))
    {
        std::cerr << "Failed to initialize ImGui OpenGL3 backend\n";
        return false;
    }
    // ---------------------------

    return true;
}

void Application::run()
{
    // Render loop
    while (!glfwWindowShouldClose(window))
    {
        // Per-frame time logic
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;

        qt_world.update(camera.Position.x, camera.Position.y, camera.Position.z);

        processInput();

        ImGui_ImplOpenGL3_NewFrame();
        ImGui_ImplGlfw_NewFrame();
        ImGui::NewFrame();

        // ---------------------------
        // Extended Debug Info Window
        // ---------------------------
        {
            ImGui::SetNextWindowSize(ImVec2(400, 300));
            ImGui::Begin("Statistics");

            // Basic stats
            ImGui::Text("Frame time: %.3f ms/frame", 1000.0f / ImGui::GetIO().Framerate);
            ImGui::Text("FPS: %.1f", ImGui::GetIO().Framerate);
            ImGui::Text("Camera Position: (%.2f, %.2f, %.2f)",
                        camera.Position.x, camera.Position.y, camera.Position.z);

            // ----------------------------------------------------------------
            // Tile and memory statistics (you might need to implement these)
            // ----------------------------------------------------------------
            // These functions are assumed to be implemented in your qt_world (or similar) class.
            int totalTiles = qt_world.getTotalTiles();     // Total number of tiles in the quadtree
            // int activeTiles = qt_world.getActiveTiles();     // Tiles currently active (or visible)
            size_t memoryUsageBytes = qt_world.getMemoryUsage(); // Memory used by the tiles (in bytes)

            ImGui::Text("Total Tiles: %d", totalTiles);
            // ImGui::Text("Active Tiles: %d", activeTiles);
            ImGui::Text("Memory Usage: %.2f MB", memoryUsageBytes / (1024.0f * 1024.0f));

            // ------------------------------------------------------
            // Graph: Memory usage history (in MB)
            // ------------------------------------------------------
            static const int historySize = 120;
            static float memUsageHistory[historySize] = { 0 };
            static int memHistoryIdx = 0;
            memUsageHistory[memHistoryIdx] = static_cast<float>(memoryUsageBytes) / (1024.0f * 1024.0f);
            memHistoryIdx = (memHistoryIdx + 1) % historySize;
            ImGui::PlotLines("Memory Usage (MB)", memUsageHistory, historySize, memHistoryIdx, nullptr, 0.0f, 10.0f, ImVec2(0, 80));

            // ------------------------------------------------------
            // Graph: Active tile count history
            // ------------------------------------------------------
            // static float activeTilesHistory[historySize] = { 0 };
            // static int activeTilesHistoryIdx = 0;
            // activeTilesHistory[activeTilesHistoryIdx] = static_cast<float>(activeTiles);
            // activeTilesHistoryIdx = (activeTilesHistoryIdx + 1) % historySize;
            // ImGui::PlotLines("Active Tiles", activeTilesHistory, historySize, activeTilesHistoryIdx, nullptr, 0.0f, static_cast<float>(totalTiles), ImVec2(0, 80));

            // ------------------------------------------------------
            // Toggle wireframe mode via ImGui checkbox (as before)
            // ------------------------------------------------------
            if (ImGui::Checkbox("Wireframe", &wireframe))
            {
                if (wireframe)
                {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Enable wireframe
                    std::cout << "Wireframe mode enabled via ImGui.\n";
                }
                else
                {
                    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Enable filled mode
                    std::cout << "Wireframe mode disabled via ImGui.\n";
                }
            }
            ImGui::End();
        }

        // ---------------------------
        // OpenGL Rendering Commands
        // ---------------------------
        glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        Shader* currentShader = terrainShaderActive ? terrainShader : shader;

        currentShader->use();

        if(terrainShaderActive) {
            currentShader->setVec3("lightDir", glm::normalize(glm::vec3(-0.2f, -1.0f, -0.3f)));
            currentShader->setVec3("lightColor", glm::vec3(1.0f, 1.0f, 1.0f));
            currentShader->setVec3("viewPos", camera.Position);
        }

        glm::mat4 model = glm::mat4(1.0f);
        currentShader->setMat4("model", model);

        glm::mat4 view = camera.GetViewMatrix();
        currentShader->setMat4("view", view);

        glm::mat4 projection = glm::perspective(
            glm::radians(camera.Zoom),
            static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
            0.1f,
            1200.0f
        );
        currentShader->setMat4("projection", projection);


        // renderer->update(qt_world.getTree(), shader->ID);
        bucket_renderer->render(&qt_world, currentShader->ID, camera.Position);
        //renderer->draw();

        // Render the ImGui frame
        ImGui::Render();
        ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

        glfwSwapBuffers(window);
        glfwPollEvents();
    }
}

void Application::processInput()
{
    // Close window if ESC is pressed
    if(glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
        glfwSetWindowShouldClose(window, true);

    // Camera movement
    if(glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
        camera.ProcessKeyboard(FORWARD, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
        camera.ProcessKeyboard(BACKWARD, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
        camera.ProcessKeyboard(LEFT, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
        camera.ProcessKeyboard(RIGHT, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_Q) == GLFW_PRESS)
        camera.ProcessKeyboard(DOWN, deltaTime);
    if(glfwGetKey(window, GLFW_KEY_E) == GLFW_PRESS)
        camera.ProcessKeyboard(UP, deltaTime);


    if(glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
    {
        if(!wireframeKeyPressed)
        {
            wireframe = !wireframe; 
            if(wireframe)
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); 
                std::cout << "Wireframe mode enabled.\n";
            }
            else
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); 
                std::cout << "Wireframe mode disabled.\n";
            }
            wireframeKeyPressed = true;
        }
    }
    if(glfwGetKey(window, GLFW_KEY_F) == GLFW_RELEASE)
    {
        wireframeKeyPressed = false;
    }
    if(glfwGetKey(window, GLFW_KEY_T) == GLFW_PRESS)
    {
        if(!terrainShaderKeyPressed)
        {
            terrainShaderActive = !terrainShaderActive; 
            if(terrainShaderActive)
            {
                std::cout << "Terrain shader enabled.\n";
            }
            else
            {
                std::cout << "Terrain shader disabled. Reverting to default shader.\n";
            }
            terrainShaderKeyPressed = true;
        }
    }
    if(glfwGetKey(window, GLFW_KEY_T) == GLFW_RELEASE)
    {
        terrainShaderKeyPressed = false;
    }

    // Deformation
    if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS)
    {
        // Compute the screen center in window coordinates.
        float screenCenterX = SCR_WIDTH / 2.0f;
        float screenCenterY = SCR_HEIGHT / 2.0f;
        
        // Prepare matrices and viewport for unprojecting.
        glm::mat4 model = glm::mat4(1.0f);
        glm::mat4 view = camera.GetViewMatrix();
        glm::mat4 projection = glm::perspective(
            glm::radians(camera.Zoom),
            static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
            0.1f,
            1200.0f
        );
        glm::vec4 viewport(0.0f, 0.0f, SCR_WIDTH, SCR_HEIGHT);
        
        // Convert from screen space to world space.
        glm::vec3 nearPoint = glm::unProject(glm::vec3(screenCenterX, SCR_HEIGHT / 2.0f, 0.0f),
                                            view * model, projection, viewport);
        glm::vec3 farPoint  = glm::unProject(glm::vec3(screenCenterX, SCR_HEIGHT / 2.0f, 1.0f),
                                            view * model, projection, viewport);
        
        glm::vec3 rayDir = glm::normalize(farPoint - nearPoint);

        float t = 0.0f;
        float tStep = 0.01f; // Adjust step size as needed
        const int maxSteps = 1000;
        bool intersectFound = false;
        glm::vec3 intersectPoint;

        float prevZ = nearPoint.z;
        float prevT = 0.0f;

        for (int i = 0; i < maxSteps; ++i) {
            glm::vec3 samplePoint = nearPoint + t * rayDir;
            float terrainZ = qt_world.getElevation(samplePoint.x, samplePoint.y);

            if (terrainZ != -1) {
                // Check if we are crossing the terrain surface
                if (prevZ > terrainZ && samplePoint.z <= terrainZ + 1e-4) {
                    intersectFound = true;

                    // **Binary search refinement** for better accuracy
                    float low = prevT, high = t;
                    for (int j = 0; j < 30; ++j) { // 10 iterations of refinement
                        float midT = (low + high) * 0.5f;
                        glm::vec3 midPoint = nearPoint + midT * rayDir;
                        float midTerrainZ = qt_world.getElevation(midPoint.x, midPoint.y);

                        if (midPoint.z > midTerrainZ)
                            low = midT;
                        else
                            high = midT;
                    }

                    // Use the refined intersection point
                    t = (low + high) * 0.5f;
                    intersectPoint = nearPoint + t * rayDir;
                    break;
                }
            }
    
            // Store previous values
            prevZ = samplePoint.z;
            prevT = t;
            
            // Advance along the ray
            t += tStep;
        }

        if (intersectFound)
        {
            // Define brush parameters.
            const float brushSize = 0.1f;  // Size (in world units) of the square brush.
            const float step = 0.01f;       // Sampling step (world units).
            const float dz = -0.05f * deltaTime; // Deformation offset.
            
            // Loop over a square region centered at the intersection point.
            for (float offsetY = -brushSize * 0.5f; offsetY <= brushSize * 0.5f; offsetY += step)
            {
                for (float offsetX = -brushSize * 0.5f; offsetX <= brushSize * 0.5f; offsetX += step)
                {
                    float deformX = intersectPoint.x + offsetX;
                    float deformY = intersectPoint.y + offsetY;

                    qt_world.deformVertex(deformX, deformY, dz);
                    
                }
            }
        }
    }

}

void Application::framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height)
{
#ifdef __APPLE__
    int fbW, fbH;
    glfwGetFramebufferSize(window, &fbW, &fbH);
    glViewport(0, 0, fbW, fbH);
#else
    glViewport(0, 0, width, height);
#endif
}

void Application::mouse_callback(double xpos, double ypos)
{
    if (firstMouse)
    {
        lastX = static_cast<float>(xpos);
        lastY = static_cast<float>(ypos);
        firstMouse = false;
    }

    float xoffset = static_cast<float>(xpos) - lastX;
    float yoffset = lastY - static_cast<float>(ypos); // Reversed since y-coordinates go from bottom to top

    lastX = static_cast<float>(xpos);
    lastY = static_cast<float>(ypos);

    camera.ProcessMouseMovement(xoffset, yoffset);
}

void Application::scroll_callback(double /*xoffset*/, double yoffset)
{
    camera.ProcessMouseScroll(static_cast<float>(yoffset));
}

void Application::setCallbacks()
{
    // Set the framebuffer size callback
    glfwSetFramebufferSizeCallback(window, 
        [](GLFWwindow* window, int width, int height) 
        {
            // Retrieve the Application instance
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            if(app)
                app->framebuffer_size_callback(window, width, height);
        }
    );

    // Set the mouse callback
    glfwSetCursorPosCallback(window, 
        [](GLFWwindow* window, double xpos, double ypos) 
        {
            // Retrieve the Application instance
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            if(app)
                app->mouse_callback(xpos, ypos);
        }
    );

    // Set the scroll callback
    glfwSetScrollCallback(window, 
        [](GLFWwindow* window, double xoffset, double yoffset) 
        {
            // Retrieve the Application instance
            Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
            if(app)
                app->scroll_callback(xoffset, yoffset);
        }
    );
}
