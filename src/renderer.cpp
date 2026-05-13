#include "renderer.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <vector>

namespace robotsim {

namespace {

std::string readFile(const std::string& path) {
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Cannot open " << path << "\n";
        return {};
    }
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

GLuint compileShader(GLenum type, const std::string& src) {
    GLuint s = glCreateShader(type);
    const char* p = src.c_str();
    glShaderSource(s, 1, &p, nullptr);
    glCompileShader(s);
    GLint ok = 0;
    glGetShaderiv(s, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetShaderInfoLog(s, sizeof(log), nullptr, log);
        std::cerr << "Shader compile error: " << log << "\n";
        glDeleteShader(s);
        return 0;
    }
    return s;
}

}  // namespace

Renderer::~Renderer() {
    if (program_)     glDeleteProgram(program_);
    if (gridProgram_) glDeleteProgram(gridProgram_);
    if (cubeVBO_)     glDeleteBuffers(1, &cubeVBO_);
    if (cubeVAO_)     glDeleteVertexArrays(1, &cubeVAO_);
    if (gridVBO_)     glDeleteBuffers(1, &gridVBO_);
    if (gridVAO_)     glDeleteVertexArrays(1, &gridVAO_);
}

bool Renderer::compileProgram(GLuint& out, const std::string& vs_src, const std::string& fs_src) {
    GLuint vs = compileShader(GL_VERTEX_SHADER, vs_src);
    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fs_src);
    if (!vs || !fs) { if (vs) glDeleteShader(vs); if (fs) glDeleteShader(fs); return false; }
    GLuint p = glCreateProgram();
    glAttachShader(p, vs);
    glAttachShader(p, fs);
    glLinkProgram(p);
    GLint ok = 0;
    glGetProgramiv(p, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[2048];
        glGetProgramInfoLog(p, sizeof(log), nullptr, log);
        std::cerr << "Program link error: " << log << "\n";
        glDeleteProgram(p);
        glDeleteShader(vs);
        glDeleteShader(fs);
        return false;
    }
    glDeleteShader(vs);
    glDeleteShader(fs);
    out = p;
    return true;
}

bool Renderer::init(const std::string& shader_dir) {
    auto vs = readFile(shader_dir + "/basic.vert");
    auto fs = readFile(shader_dir + "/basic.frag");
    if (vs.empty() || fs.empty()) return false;
    if (!compileProgram(program_, vs, fs)) return false;

    uModel_    = glGetUniformLocation(program_, "uModel");
    uView_     = glGetUniformLocation(program_, "uView");
    uProj_     = glGetUniformLocation(program_, "uProj");
    uColor_    = glGetUniformLocation(program_, "uColor");
    uLightDir_ = glGetUniformLocation(program_, "uLightDir");
    uViewPos_  = glGetUniformLocation(program_, "uViewPos");

    auto gvs = readFile(shader_dir + "/grid.vert");
    auto gfs = readFile(shader_dir + "/grid.frag");
    if (gvs.empty() || gfs.empty()) return false;
    if (!compileProgram(gridProgram_, gvs, gfs)) return false;
    gridView_  = glGetUniformLocation(gridProgram_, "uView");
    gridProj_  = glGetUniformLocation(gridProgram_, "uProj");
    gridColor_ = glGetUniformLocation(gridProgram_, "uColor");

    setupCube();
    setupGrid(4.0f, 0.25f);
    return true;
}

void Renderer::setupCube() {
    // Position (x,y,z) + Normal (nx,ny,nz). 6 faces * 2 tris * 3 verts = 36 verts.
    static const float v[] = {
        // +X
         0.5f,-0.5f,-0.5f,  1,0,0,    0.5f, 0.5f,-0.5f,  1,0,0,    0.5f, 0.5f, 0.5f,  1,0,0,
         0.5f,-0.5f,-0.5f,  1,0,0,    0.5f, 0.5f, 0.5f,  1,0,0,    0.5f,-0.5f, 0.5f,  1,0,0,
        // -X
        -0.5f,-0.5f, 0.5f, -1,0,0,   -0.5f, 0.5f, 0.5f, -1,0,0,   -0.5f, 0.5f,-0.5f, -1,0,0,
        -0.5f,-0.5f, 0.5f, -1,0,0,   -0.5f, 0.5f,-0.5f, -1,0,0,   -0.5f,-0.5f,-0.5f, -1,0,0,
        // +Y
        -0.5f, 0.5f,-0.5f,  0,1,0,    0.5f, 0.5f,-0.5f,  0,1,0,    0.5f, 0.5f, 0.5f,  0,1,0,
        -0.5f, 0.5f,-0.5f,  0,1,0,    0.5f, 0.5f, 0.5f,  0,1,0,   -0.5f, 0.5f, 0.5f,  0,1,0,
        // -Y
        -0.5f,-0.5f, 0.5f,  0,-1,0,   0.5f,-0.5f, 0.5f,  0,-1,0,   0.5f,-0.5f,-0.5f,  0,-1,0,
        -0.5f,-0.5f, 0.5f,  0,-1,0,   0.5f,-0.5f,-0.5f,  0,-1,0,  -0.5f,-0.5f,-0.5f,  0,-1,0,
        // +Z
        -0.5f,-0.5f, 0.5f,  0,0,1,    0.5f,-0.5f, 0.5f,  0,0,1,    0.5f, 0.5f, 0.5f,  0,0,1,
        -0.5f,-0.5f, 0.5f,  0,0,1,    0.5f, 0.5f, 0.5f,  0,0,1,   -0.5f, 0.5f, 0.5f,  0,0,1,
        // -Z
        -0.5f, 0.5f,-0.5f,  0,0,-1,   0.5f, 0.5f,-0.5f,  0,0,-1,   0.5f,-0.5f,-0.5f,  0,0,-1,
        -0.5f, 0.5f,-0.5f,  0,0,-1,   0.5f,-0.5f,-0.5f,  0,0,-1,  -0.5f,-0.5f,-0.5f,  0,0,-1,
    };
    glGenVertexArrays(1, &cubeVAO_);
    glGenBuffers(1, &cubeVBO_);
    glBindVertexArray(cubeVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, cubeVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(v), v, GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);
}

void Renderer::setupGrid(float size, float cell) {
    std::vector<float> verts;
    int n = static_cast<int>(size / cell);
    for (int i = -n; i <= n; ++i) {
        float c = i * cell;
        verts.insert(verts.end(), {-size, c, 0.0f,  size, c, 0.0f});
        verts.insert(verts.end(), {c, -size, 0.0f,  c,  size, 0.0f});
    }
    gridVertCount_ = static_cast<int>(verts.size() / 3);

    glGenVertexArrays(1, &gridVAO_);
    glGenBuffers(1, &gridVBO_);
    glBindVertexArray(gridVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 3 * sizeof(float), (void*)0);
    glBindVertexArray(0);
}

void Renderer::beginFrame(int fb_w, int fb_h,
                          const glm::mat4& view, const glm::mat4& proj,
                          const glm::vec3& camera_pos) {
    glViewport(0, 0, fb_w, fb_h);
    glEnable(GL_DEPTH_TEST);
    glClearColor(0.10f, 0.11f, 0.13f, 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    view_    = view;
    proj_    = proj;
    viewPos_ = camera_pos;
}

void Renderer::drawBox(const glm::mat4& model, const glm::vec3& color) {
    glUseProgram(program_);
    glUniformMatrix4fv(uModel_, 1, GL_FALSE, &model[0][0]);
    glUniformMatrix4fv(uView_,  1, GL_FALSE, &view_[0][0]);
    glUniformMatrix4fv(uProj_,  1, GL_FALSE, &proj_[0][0]);
    glUniform3fv(uColor_, 1, &color.x);
    glm::vec3 light = glm::normalize(glm::vec3(-0.3f, -0.4f, -1.0f));
    glUniform3fv(uLightDir_, 1, &light.x);
    glUniform3fv(uViewPos_,  1, &viewPos_.x);
    glBindVertexArray(cubeVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 36);
}

void Renderer::drawGround() {
    glUseProgram(gridProgram_);
    glUniformMatrix4fv(gridView_, 1, GL_FALSE, &view_[0][0]);
    glUniformMatrix4fv(gridProj_, 1, GL_FALSE, &proj_[0][0]);
    glm::vec3 col(0.30f, 0.33f, 0.40f);
    glUniform3fv(gridColor_, 1, &col.x);
    glBindVertexArray(gridVAO_);
    glDrawArrays(GL_LINES, 0, gridVertCount_);
}

}  // namespace robotsim
