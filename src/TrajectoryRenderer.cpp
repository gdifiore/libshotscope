#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <vector>
#include <cmath>

class TrajectoryRenderer {
    public:
        TrajectoryRenderer() : panX(0.0f), panY(0.0f) {}

        void init(GLFWwindow* window) {
            glfwSetMouseButtonCallback(window, mouse_callback);
            // ... (initialize GLEW, set up vertex data, etc.)
        }

        void render() {
            // Clear color buffer
            glClear(GL_COLOR_BUFFER_BIT);

            // Set up projection and modelview matrices
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            glOrtho(-1.0, 1.0, -1.0, 1.0, -1.0, 1.0);
            glMatrixMode(GL_MODELVIEW);
            glLoadIdentity();
            glTranslatef(panX, panY, 0.0f); // Apply panning transformation

            // Render vertices as a line loop
            glBegin(GL_LINE_LOOP);
            for (int i = 0; i < static_cast<int>(vertices.size()); i += 3) {
                glVertex3f(vertices[i], vertices[i + 1], vertices[i + 2]);
            }
            glEnd();
        }

    private:
        std::vector<GLfloat> vertices;
        float panX, panY;

        static void mouse_callback(GLFWwindow* window, int button, int action, int mod) {
            if (button == GLFW_MOUSE_BUTTON_LEFT && action == GLFW_PRESS) {
                double xpos, ypos;
                glfwGetCursorPos(window, &xpos, &ypos);

                double prevXpos, prevYpos;
                glfwGetCursorPos(window, &prevXpos, &prevYpos);

                TrajectoryRenderer* renderer = static_cast<TrajectoryRenderer*>(glfwGetWindowUserPointer(window));
                renderer->panX += static_cast<float>(xpos - prevXpos) / 100.0f;
                renderer->panY -= static_cast<float>(ypos - prevYpos) / 100.0f;
            }
        }
};