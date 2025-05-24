// Camera.h  (header‑only, drop‑in)  ──────────────────────────
#pragma once
#include <glm/glm.hpp>
#include <algorithm>
#include <cmath>

#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtx/quaternion.hpp>

/* -----------------------------------------------------------------------
   Split double → high/low floats  (useful if you ship the eye to shaders)
   -------------------------------------------------------------------- */
inline void splitDouble(double v, float& hi, float& lo)
{
    constexpr double magic = double(1ull << 25) + 1.0;
    double t = magic * v;
    hi = static_cast<float>(t - (t - v));
    lo = static_cast<float>(v - hi);
}

/* -----------------------------------------------------------------------
   High‑precision free‑fly camera, quaternion‑based
   -------------------------------------------------------------------- */
enum Camera_Movement { FORWARD, BACKWARD, LEFT, RIGHT, UP, DOWN };

class Camera
{
public:
    /* public state ------------------------------------------------------ */
    glm::dvec3 Position { 0.0, 0.0, 0.0 };
    glm::dvec3 Front    { 0.0, 0.0,-1.0 };
    glm::dvec3 Up       { 0.0, 1.0, 0.0 };
    glm::dvec3 Right    { 1.0, 0.0, 0.0 };

    double Zoom = 45.0;          // fov (°)

    /* ctor -------------------------------------------------------------- */
    explicit Camera(glm::dvec3 startPos)
        : Position(startPos)
    {}

    /* view matrix ------------------------------------------------------- */
    glm::mat4 GetViewMatrix() const
    {
        glm::dvec3 center = Position + Front;
        return glm::mat4(glm::lookAt(Position, center, Up));
    }

    /* movement ---------------------------------------------------------- */
    void ProcessKeyboard(Camera_Movement dir, double dt);
    void ProcessMouseMovement(double dx, double dy);
    void ProcessMouseScroll(double dZoom);
    void ProcessRoll(double dRoll);

private:
    /* internal orientation --------------------------------------------- */
    glm::dquat orientation_ { 1.0, 0.0, 0.0, 0.0 };   // w,x,y,z

    /* tweakables */
    static constexpr double kSpeed      = 50000.0;  // m/s
    static constexpr double kMouseSens  = 0.1;   // deg / pixel
    static constexpr double kRollSens   = 0.1;   // deg / tap

    /* helpers */
    void applyQuaternion(const glm::dquat& q) { orientation_ = glm::normalize(q * orientation_); }
    void refreshAxes();                        // updates Front/Right/Up from orientation_
};

/* -----------------------------------------------------------------------
   implementation
   -------------------------------------------------------------------- */
inline void Camera::refreshAxes()
{
    Front = glm::normalize(orientation_ * glm::dvec3( 0, 0,-1 ));
    Right = glm::normalize(orientation_ * glm::dvec3( 1, 0, 0 ));
    Up    = glm::normalize(orientation_ * glm::dvec3( 0, 1, 0 ));
}

inline void Camera::ProcessKeyboard(Camera_Movement dir, double dt)
{
    double v = kSpeed * dt;
    if (dir == FORWARD)  Position += Front  * v;
    if (dir == BACKWARD) Position -= Front  * v;
    if (dir == LEFT)     Position -= Right  * v;
    if (dir == RIGHT)    Position += Right  * v;
    if (dir == UP)       Position += Up     * v;
    if (dir == DOWN)     Position -= Up     * v;
}

inline void Camera::ProcessMouseMovement(double dx, double dy)
{
    double yawDeg   =  -dx * kMouseSens;
    double pitchDeg =  dy * kMouseSens;

    /* relative axes:  yaw ⟳ Up,  pitch ⟳ Right */
    glm::dquat qYaw   = glm::angleAxis( glm::radians(yawDeg),   Up    );
    glm::dquat qPitch = glm::angleAxis( glm::radians(pitchDeg), Right );

    applyQuaternion(qYaw);
    applyQuaternion(qPitch);
    refreshAxes();
}

inline void Camera::ProcessRoll(double dRoll)
{
    glm::dquat qRoll = glm::angleAxis(glm::radians(dRoll * kRollSens), Front);
    applyQuaternion(qRoll);
    refreshAxes();
}

inline void Camera::ProcessMouseScroll(double dZoom)
{
    Zoom = std::clamp(Zoom - dZoom, 1.0, 75.0);
}
