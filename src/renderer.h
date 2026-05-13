#pragma once
#if defined(__APPLE__)
    #define GL_SILENCE_DEPRECATION
    #include <OpenGL/gl3.h>
    #include <OpenGL/gl3ext.h>
#else
    #define GL_GLEXT_PROTOTYPES
    #include <GL/gl.h>
    #include <GL/glext.h>
#endif
#include <glm/glm.hpp>
#include <string>

namespace robotsim {

class Renderer {
public:
    Renderer() = default;
    ~Renderer();
    Renderer(const Renderer&) = delete;
    Renderer& operator=(const Renderer&) = delete;

    bool init(const std::string& shader_dir);

    void beginFrame(int fb_w, int fb_h,
                    const glm::mat4& view, const glm::mat4& proj,
                    const glm::vec3& camera_pos);

    // Unit cube centered at origin, transformed by model.
    void drawBox(const glm::mat4& model, const glm::vec3& color);

    // Draws an XY-plane grid at z=0.
    void drawGround();

private:
    GLuint program_     = 0;
    GLuint gridProgram_ = 0;

    GLuint cubeVAO_ = 0, cubeVBO_ = 0;
    GLuint gridVAO_ = 0, gridVBO_ = 0;
    int    gridVertCount_ = 0;

    GLint uModel_ = -1, uView_ = -1, uProj_ = -1;
    GLint uColor_ = -1, uLightDir_ = -1, uViewPos_ = -1;
    GLint gridView_ = -1, gridProj_ = -1, gridColor_ = -1;

    glm::mat4 view_{1.0f}, proj_{1.0f};
    glm::vec3 viewPos_{0.0f};

    bool   compileProgram(GLuint& out, const std::string& vs, const std::string& fs);
    void   setupCube();
    void   setupGrid(float size, float cell);
};

}  // namespace robotsim
