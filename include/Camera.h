#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

// Defines possible camera movement directions.
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// Default camera values for a Z-up system.
// Here, we set YAW to 0 so that with zero pitch the camera faces along the positive X-axis.
const float YAW         = 0.0f;   
const float PITCH       = 0.0f;   // 0 degrees means level (no tilt up or down)
const float SPEED       = 20.0f;
const float SENSITIVITY = 0.05f;
const float ZOOM        = 45.0f;

class Camera
{
public:
    // Camera Attributes
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;
    // Euler Angles
    float Yaw;
    float Pitch;
    // Camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Constructor with vectors.
    // Note: The default up vector is now (0, 0, 1) to make Z the upward direction.
    Camera(
        glm::vec3 position = glm::vec3(0.0f, 50.0f, 100.0f),
        glm::vec3 up       = glm::vec3(0.0f, 0.0f, 1.0f),
        float yaw          = YAW,
        float pitch        = PITCH
    ) : Front(glm::vec3(1.0f, 0.0f, 0.0f)),  // With yaw=0 and pitch=0, the camera looks along +X.
        MovementSpeed(SPEED),
        MouseSensitivity(SENSITIVITY),
        Zoom(ZOOM)
    {
        Position = position;
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        updateCameraVectors();
    }

    // Returns the view matrix calculated using LookAt.
    glm::mat4 GetViewMatrix()
    {
        return glm::lookAt(Position, Position + Front, Up);
    }

    // Processes keyboard input.
    void ProcessKeyboard(Camera_Movement direction, float deltaTime)
    {
        float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)
            Position += Front * velocity;
        if (direction == BACKWARD)
            Position -= Front * velocity;
        if (direction == LEFT)
            Position -= Right * velocity;
        if (direction == RIGHT)
            Position += Right * velocity;
        if (direction == UP)
            Position += WorldUp * velocity;
        if (direction == DOWN)
            Position -= WorldUp * velocity;
    }

    // Processes mouse movement input.
    void ProcessMouseMovement(float xoffset, float yoffset, bool constrainPitch = true)
    {
        xoffset *= -MouseSensitivity;
        yoffset *= MouseSensitivity;

        Yaw   += xoffset;
        Pitch += yoffset;

        // Constrain the pitch angle to prevent flipping.
        if (constrainPitch)
        {
            if (Pitch > 89.0f)
                Pitch = 89.0f;
            if (Pitch < -89.0f)
                Pitch = -89.0f;
        }

        // Update Front, Right, and Up vectors.
        updateCameraVectors();
    }

    // Processes mouse scroll input.
    void ProcessMouseScroll(float yoffset)
    {
        Zoom -= yoffset;
        if (Zoom < 1.0f)
            Zoom = 1.0f;
        if (Zoom > 90.0f)
            Zoom = 90.0f;
    }

private:
    // Updates the camera's vectors based on the updated Euler angles.
    // In a Z-up coordinate system, the standard formulas become:
    //    front.x = cos(yaw) * cos(pitch)
    //    front.y = sin(yaw) * cos(pitch)
    //    front.z = sin(pitch)
    void updateCameraVectors()
    {
        glm::vec3 front;
        front.x = cos(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.y = sin(glm::radians(Yaw)) * cos(glm::radians(Pitch));
        front.z = sin(glm::radians(Pitch));
        Front = glm::normalize(front);

        // Recalculate Right and Up vectors.
        // Right vector: perpendicular to both Front and WorldUp.
        Right = glm::normalize(glm::cross(Front, WorldUp));
        // Up vector: perpendicular to both Right and Front.
        Up = glm::normalize(glm::cross(Right, Front));
    }
};

#endif
