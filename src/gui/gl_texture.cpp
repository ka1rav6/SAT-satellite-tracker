#include "gui/gl_texture.hpp"

#include <GL/gl.h>

namespace sat::gui {

GlTexture::~GlTexture() {
    if (tex_) {
        const GLuint t = tex_;
        glDeleteTextures(1, &t);
    }
}

void GlTexture::upload_grey(std::span<const uint8_t> pixels, int width, int height) {
    if (width <= 0 || height <= 0) return;
    const size_t need = static_cast<size_t>(width) * static_cast<size_t>(height);
    if (pixels.size() < need) return;

    const bool resized = (width != w_ || height != h_);
    if (resized) {
        rgba_.assign(need * 4, 255);
        w_ = width;
        h_ = height;
    }

    // Grey -> RGBA. Alpha is left at 255 by the assign above and never touched,
    // so this writes three bytes per pixel rather than four.
    for (size_t i = 0; i < need; ++i) {
        const uint8_t v = pixels[i];
        uint8_t* d = &rgba_[i * 4];
        d[0] = v; d[1] = v; d[2] = v;
    }

    if (tex_ == 0) {
        GLuint t = 0;
        glGenTextures(1, &t);
        tex_ = t;
        glBindTexture(GL_TEXTURE_2D, tex_);
        // NEAREST, not LINEAR. This is a scientific image: a viewer zooming in
        // to check a centroid needs to see the actual pixels, and interpolation
        // would invent sub-pixel structure that is not in the data.
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    } else {
        glBindTexture(GL_TEXTURE_2D, tex_);
    }

    // Rows are tightly packed; the default of 4 happens to be right for RGBA
    // but saying so makes the code independent of that coincidence.
    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);

    if (resized) {
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w_, h_, 0,
                     GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
    } else {
        // Reuses the existing storage — no reallocation per frame.
        glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, w_, h_,
                        GL_RGBA, GL_UNSIGNED_BYTE, rgba_.data());
    }
}

}  // namespace sat::gui
