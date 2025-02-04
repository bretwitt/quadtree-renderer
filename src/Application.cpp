// Application.cpp
#include "Application.h"
#include <iostream>

// Heightfield parameters
const int DEFAULT_GRID_SIZE = 100;
const float DEFAULT_GRID_SCALE = 1.5f;
const float DEFAULT_HEIGHT_SCALE = 10.0f;

// Constructor
Application::Application(int width, int height, const char* title)
    : SCR_WIDTH(width), SCR_HEIGHT(height), windowTitle(title), window(nullptr),
      shader(nullptr), heightfield(nullptr), 
      camera(glm::vec3(0.0f, 50.0f, 100.0f)), // Initial camera position
      lastX(width / 2.0f), lastY(height / 2.0f),
      firstMouse(true), deltaTime(0.0f), lastFrame(0.0f),
      wireframe(false), wireframeKeyPressed(false),
      qt(QuadtreeTile<int>(0.0f,0.0f,100.0f,100.0f)),
      renderer(nullptr),
      bucket_renderer(nullptr)
{

    if(!init())
    {
        std::cerr << "Failed to initialize Application\n";
        exit(EXIT_FAILURE);
    }
}

Application::~Application()
{
    // Cleanup
    delete shader;
    delete heightfield;
    delete renderer;
    delete bucket_renderer;
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

    // Set GLFW window hints for OpenGL version (3.3 Core)
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

    // For MacOS
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

    // Set viewport
    glViewport(0, 0, SCR_WIDTH, SCR_HEIGHT);

    // Set user pointer to this instance for callbacks
    glfwSetWindowUserPointer(window, this);

    // Set callbacks
    setCallbacks();

    // Capture the mouse
    glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

    // Enable depth testing
    glEnable(GL_DEPTH_TEST);

    // Initialize Shader
    shader = new Shader("../shaders/vertex_shader.glsl", "../shaders/fragment_shader.glsl");

    // Initialize Heightfield
    // heightfield = new Heightfield(DEFAULT_GRID_SIZE, DEFAULT_GRID_SCALE, DEFAULT_HEIGHT_SCALE);

    // qt = QuadtreeTile<int>(0.0f,0.0f,100.0f,100.0f);
    // qt.insert({ -30.0f, -30.0f, 1 });
    // qt.insert({ -35.0f, -30.0f, 1 });
    // qt.insert({ -33.0f, -30.0f, 1 });
    // qt.insert({  30.0f, -30.0f, 2 });
    // qt.insert({  30.0f,  30.0f, 3 });
    // qt.insert({ -30.0f,  30.0f, 4 });

    // std::cout << qt.getBoundary().width << std::endl;
    renderer = new QuadtreeRenderer();
    bucket_renderer = new BucketMeshRenderer();

    qt.getTree()->setScale(2);
    // qt.getTree()->getLevel();

    return true;
}

void Application::run()
{
    // Render loop
    while(!glfwWindowShouldClose(window))
    {
        // Per-frame time logic
        float currentFrame = static_cast<float>(glfwGetTime());
        deltaTime = currentFrame - lastFrame;
        lastFrame = currentFrame;
        
        // float dist = glm::distance(glm::vec2(qt.getBoundary().x, qt.getBoundary().y), glm::vec2(camera.Position.x, camera.Position.z));
        // int metric = fmax(fmin(5*(100./dist),5.0),1.0);

        // std::cout << "--- View Level: " <<  metric << std::endl;
        // qt.setScale(metric);
        
        int subdivisions = 0.0;
        qt.updateLOD(camera.Position.x,camera.Position.y,camera.Position.z,120.0f, 120.0f, subdivisions);

        if(subdivisions != 0) {
            std::cout << "--- Subdivisions " << subdivisions << std::endl;

        }

        processInput();

        glClearColor(0.2f,0.3f,0.3f,1.0f);
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        shader->use();

        renderer->update(qt.getTree(), shader->ID);
        bucket_renderer->render(qt.getMeshes(), shader->ID);

        glm::mat4 model = glm::mat4(1.0f);
        shader->setMat4("model", model);

        glm::mat4 view = camera.GetViewMatrix();
        shader->setMat4("view", view);

        glm::mat4 projection = glm::perspective(
            glm::radians(camera.Zoom),
            static_cast<float>(SCR_WIDTH) / static_cast<float>(SCR_HEIGHT),
            0.1f,
            1000.0f
        );
        shader->setMat4("projection", projection);

        renderer->draw();

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

    // Wireframe toggle with the F key
    if(glfwGetKey(window, GLFW_KEY_F) == GLFW_PRESS)
    {
        if(!wireframeKeyPressed)
        {
            wireframe = !wireframe; // Toggle the wireframe state
            if(wireframe)
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_LINE); // Enable wireframe
                std::cout << "Wireframe mode enabled.\n";
            }
            else
            {
                glPolygonMode(GL_FRONT_AND_BACK, GL_FILL); // Enable filled mode
                std::cout << "Wireframe mode disabled.\n";
            }

            wireframeKeyPressed = true; // Mark the key as pressed
        }
    }

    // Reset the key press state when the key is released
    if(glfwGetKey(window, GLFW_KEY_F) == GLFW_RELEASE)
    {
        wireframeKeyPressed = false;
    }
}

void Application::framebuffer_size_callback(GLFWwindow* /*window*/, int width, int height)
{
    // Adjust the viewport when the window size changes
    glViewport(0, 0, width, height);
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
    // glfwSetCursorPosCallback(window, 
    //     [](GLFWwindow* window, double xpos, double ypos) 
    //     {
    //         // Retrieve the Application instance
    //         Application* app = static_cast<Application*>(glfwGetWindowUserPointer(window));
    //         if(app)
    //             app->mouse_callback(xpos, ypos);
    //     }
    // );

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
