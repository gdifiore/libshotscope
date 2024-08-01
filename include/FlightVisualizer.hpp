#ifndef FLIGHTVISUALIZER_h
#define FLIGHTVISUALIZER_h

#include <vector>
#include <GL/glew.h>
#include <GLFW/glfw3.h>

#include "math_utils.hpp"

class FlightVisualizer
{
public:
    FlightVisualizer(int width, int height, const char *title);
    ~FlightVisualizer();

    void setTrajectory(const std::vector<Vector3D> &trajectory);
    void run();

private:
    int m_width;
    int m_height;
    const char *m_title;
    GLFWwindow *m_window;
    GLuint m_shaderProgram;
    GLuint m_VBO, m_VAO;
    std::vector<Vector3D> m_trajectory;

    const char *vertexShaderSource = R"(
        #version 330 core
        layout (location = 0) in vec3 aPos;
        uniform mat4 mvp;
        void main()
        {
            gl_Position = mvp * vec4(aPos.x, aPos.y, aPos.z, 1.0);
        }
    )";

    const char *fragmentShaderSource = R"(
        #version 330 core
        out vec4 FragColor;
        void main()
        {
            FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
        }
    )";

    void initializeGLFW();
    void initializeGLEW();
    void createShaders();
    void setupBuffers();
    void updateBuffers();
    void render();
    void cleanup();
};

#endif // FLIGHTVISUALIZER_H