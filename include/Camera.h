#ifndef CAMERA_H
#define CAMERA_H

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/quaternion.hpp>

// Camera movement directions
enum Camera_Movement {
    FORWARD,
    BACKWARD,
    LEFT,
    RIGHT,
    UP,
    DOWN
};

// Default camera constants
const float YAW         = 0.0f;
const float PITCH       = 0.0f;
const float ROLL        = 0.0f;
const float SPEED       = 100.0f;
const float SENSITIVITY = 0.05f;
const float ZOOM        = 45.0f;

class Camera {
public:
    // Camera orientation
    glm::quat Orientation;

    // Camera attributes
    glm::vec3 Position;
    glm::vec3 Front;
    glm::vec3 Up;
    glm::vec3 Right;
    glm::vec3 WorldUp;

    // Euler angles
    float Yaw;
    float Pitch;
    float Roll;

    // Camera options
    float MovementSpeed;
    float MouseSensitivity;
    float Zoom;

    // Constructor
    Camera(
        glm::vec3 position = glm::vec3(0.0f, 50.0f, 100.0f),
        glm::vec3 up       = glm::vec3(0.0f, 0.0f, 1.0f),
        float yaw          = YAW,
        float pitch        = PITCH,
        float roll         = ROLL
    ) : Front(glm::vec3(1.0f, 0.0f, 0.0f)),
        MovementSpeed(SPEED),
        MouseSensitivity(SENSITIVITY),
        Zoom(ZOOM)
    {
        Position = position;
        WorldUp = up;
        Yaw = yaw;
        Pitch = pitch;
        Roll = roll;

        Orientation = glm::quat(glm::vec3(glm::radians(Pitch), glm::radians(Yaw), glm::radians(Roll)));
        updateCameraVectors();
    }

    // Returns the view matrix using LookAt
    glm::mat4 GetViewMatrix() {
        return glm::lookAt(Position, Position + Front, Up);
    }

    // Processes keyboard input
    void ProcessKeyboard(Camera_Movement direction, float deltaTime) {
        float velocity = MovementSpeed * deltaTime;
        if (direction == FORWARD)  Position += Front * velocity;
        if (direction == BACKWARD) Position -= Front * velocity;
        if (direction == LEFT)     Position -= Right * velocity;
        if (direction == RIGHT)    Position += Right * velocity;
        if (direction == UP)       Position += Up * velocity;
        if (direction == DOWN)     Position -= Up * velocity;
    }


    void ProcessMouseMovement(float xoffset, float yoffset) {
        xoffset *= MouseSensitivity;
        yoffset *= MouseSensitivity;

        glm::quat pitchQuat = glm::angleAxis(glm::radians(-yoffset), Right); // Local pitch
        glm::quat yawQuat   = glm::angleAxis(glm::radians(-xoffset), Up);    // Local yaw

        Orientation = glm::normalize(yawQuat * pitchQuat * Orientation);
        updateCameraVectors();
    }

    // Processes mouse scroll input for speed adjustment
    void ProcessMouseScroll(float yoffset) {
        MovementSpeed += yoffset;
        MovementSpeed = glm::clamp(MovementSpeed, 0.05f, 100.0f);
    }

    // Processes roll input (in degrees)
    void ProcessRoll(float rollOffset, float deltaTime = 0.1f) {
        float angle = glm::radians(rollOffset * deltaTime);
        glm::quat rollQuat = glm::angleAxis(angle, Front);
        Orientation = glm::normalize(rollQuat * Orientation);
        updateCameraVectors();
    }

private:
    // Updates the camera's direction vectors based on orientation quaternion
    void updateCameraVectors() {
        Front = glm::normalize(Orientation * glm::vec3(0.0f, 0.0f, -1.0f));
        Right = glm::normalize(Orientation * glm::vec3(1.0f, 0.0f, 0.0f));
        Up    = glm::normalize(glm::cross(Right, Front));
    }
};

#endif // CAMERA_H
