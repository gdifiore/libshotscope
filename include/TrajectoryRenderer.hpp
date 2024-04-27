#ifndef OPENGL_RENDERER_HPP
#define OPENGL_RENDERER_HPP

#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>

class TrajectoryRenderer {
    public:
        TrajectoryRenderer();

        void init(GLFWwindow* window);
        void render(GLFWwindow* window);

    private:
        std::vector<GLfloat> vertices;
        float panX, panY;

        static void mouse_callback(GLFWwindow* window, int button, int action, int mod)
};

#endif // OPENGL_RENDERER_HPP