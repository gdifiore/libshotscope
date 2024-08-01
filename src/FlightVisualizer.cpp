#include "FlightVisualizer.hpp"
#include "math_utils.hpp"

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <array>
#include <iostream>
#include <stdexcept>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

FlightVisualizer::FlightVisualizer(int width, int height, const char *title)
    : m_width(width), m_height(height), m_title(title)
{
    std::cout << "FlightVisualizer constructor called." << std::endl;
    initializeGLFW();
    initializeGLEW();
    createShaders();
    setupBuffers();
    std::cout << "FlightVisualizer initialization completed." << std::endl;
}

FlightVisualizer::~FlightVisualizer()
{
    cleanup();
}

void FlightVisualizer::setTrajectory(const std::vector<Vector3D> &trajectory)
{
    m_trajectory = trajectory;

    updateBuffers();
}

void FlightVisualizer::run()
{
    // Create and set up MVP matrix
    glm::mat4 mvp = glm::ortho(-10.0f, 10.0f, -10.0f, 10.0f, -10.0f, 10.0f);
    GLuint mvpLocation = glGetUniformLocation(m_shaderProgram, "mvp");

    while (!glfwWindowShouldClose(m_window))
    {
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

        glUseProgram(m_shaderProgram);

        // Set MVP matrix
        glUniformMatrix4fv(mvpLocation, 1, GL_FALSE, glm::value_ptr(mvp));

        glBindVertexArray(m_VAO);

        glDrawArrays(GL_LINE_STRIP, 0, m_trajectory.size());

        glBindVertexArray(0);

        glfwSwapBuffers(m_window);
        glfwPollEvents();
    }
}

void FlightVisualizer::initializeGLFW()
{
    if (!glfwInit())
    {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    m_window = glfwCreateWindow(m_width, m_height, m_title, NULL, NULL);
    if (!m_window)
    {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }

    glfwMakeContextCurrent(m_window);
}

void FlightVisualizer::initializeGLEW()
{
    std::cout << "Initializing GLEW..." << std::endl;
    glfwMakeContextCurrent(m_window);
    if (glewInit() != GLEW_OK)
    {
        throw std::runtime_error("Failed to initialize GLEW");
    }
    std::cout << "GLEW initialized successfully." << std::endl;
}

void FlightVisualizer::createShaders()
{
    std::cout << "Creating shaders..." << std::endl;
    GLuint vertexShader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertexShader, 1, &vertexShaderSource, NULL);
    glCompileShader(vertexShader);

    GLint success;
    GLchar infoLog[512];
    glGetShaderiv(vertexShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(vertexShader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::VERTEX::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    GLuint fragmentShader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragmentShader, 1, &fragmentShaderSource, NULL);
    glCompileShader(fragmentShader);
    glGetShaderiv(fragmentShader, GL_COMPILE_STATUS, &success);
    if (!success) {
        glGetShaderInfoLog(fragmentShader, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::FRAGMENT::COMPILATION_FAILED\n" << infoLog << std::endl;
    }

    m_shaderProgram = glCreateProgram();
    glAttachShader(m_shaderProgram, vertexShader);
    glAttachShader(m_shaderProgram, fragmentShader);
    glLinkProgram(m_shaderProgram);
    glGetProgramiv(m_shaderProgram, GL_LINK_STATUS, &success);
    if (!success) {
        glGetProgramInfoLog(m_shaderProgram, 512, NULL, infoLog);
        std::cerr << "ERROR::SHADER::PROGRAM::LINKING_FAILED\n" << infoLog << std::endl;
    }

    glDeleteShader(vertexShader);
    glDeleteShader(fragmentShader);
    std::cout << "Shaders created successfully." << std::endl;
}


void FlightVisualizer::setupBuffers()
{
    std::cout << "Setting up buffers..." << std::endl;
    glGenVertexArrays(1, &m_VAO);
    glGenBuffers(1, &m_VBO);

    glBindVertexArray(m_VAO);
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);

    // Vertex attribute pointer setup for Vector3D (std::array<float, 3>)
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);

    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindVertexArray(0);
    std::cout << "Buffers set up successfully." << std::endl;
}

void FlightVisualizer::updateBuffers()
{
    std::cout << "Updating buffers with trajectory data..." << std::endl;
    glBindBuffer(GL_ARRAY_BUFFER, m_VBO);
    glBufferData(GL_ARRAY_BUFFER, m_trajectory.size() * sizeof(Vector3D), m_trajectory.data(), GL_STATIC_DRAW);

    // Check for errors
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error in updateBuffers(): " << error << std::endl;
    }

    std::cout << "First few trajectory points:" << std::endl;
    for (size_t i = 0; i < std::min(m_trajectory.size(), size_t(5)); ++i) {
        std::cout << m_trajectory[i][0] << ", " << m_trajectory[i][1] << ", " << m_trajectory[i][2] << std::endl;
    }
    std::cout << "Buffers updated successfully." << std::endl;
}


void FlightVisualizer::render()
{
    if (m_trajectory.empty()) {
        std::cerr << "No trajectory data to render." << std::endl;
        return;
    }

    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glUseProgram(m_shaderProgram);

    glBindVertexArray(m_VAO);

    // Check if VAO is correctly bound
    GLenum error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after glBindVertexArray: " << error << std::endl;
    }

    glDrawArrays(GL_LINE_STRIP, 0, m_trajectory.size());

    // Check for errors after drawing
    error = glGetError();
    if (error != GL_NO_ERROR) {
        std::cerr << "OpenGL error after glDrawArrays: " << error << std::endl;
    }
}

void FlightVisualizer::cleanup()
{
    glDeleteVertexArrays(1, &m_VAO);
    glDeleteBuffers(1, &m_VBO);
    glDeleteProgram(m_shaderProgram);
    glfwTerminate();
}