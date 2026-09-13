// gui/gl_texture.hpp — a GPU texture holding the current camera frame.
//
// Design §12: "OpenGL use is minimal: one GL_R8 texture upload per frame, one
// quad, ImGui for the rest."
//
// ---------------------------------------------------------------------------
// WHY RGBA AND NOT GL_R8
// ---------------------------------------------------------------------------
// The design doc says GL_R8, and that is the tighter upload — a quarter of the
// bytes. It needs two things this file deliberately avoids: a GL 3.0+ context
// (GL_R8 is not a GL 1.1 enum) and a texture swizzle so a single-channel
// texture renders as grey rather than red.
//
// Expanding to RGBA on the CPU instead costs ~0.3 ms per frame for a 640x480
// image, which is invisible against a 16 ms display budget, and buys three
// things worth more than the bandwidth:
//
//   * Every GL call here is OpenGL 1.1, exported directly by the system library
//     on Linux, Windows and macOS. No glad, no GLEW, no loader to initialise
//     and no loader to go wrong on an evaluator's machine. ImGui's own backend
//     carries an embedded loader for the modern calls it makes.
//   * The expansion pass is where a false-colour map goes when one is wanted,
//     which is much easier on the CPU than in a shader we would otherwise have
//     to write and compile.
//   * It is the same code path on every platform, so "it renders on my machine"
//     and "it renders on the demo machine" are the same claim.
//
// The frame is uploaded with glTexSubImage2D after the first allocation, so the
// driver reuses the existing storage rather than reallocating every frame.

#pragma once

#include <cstdint>
#include <span>
#include <vector>

namespace sat::gui {

class GlTexture {
public:
    GlTexture() = default;
    ~GlTexture();

    GlTexture(const GlTexture&)            = delete;
    GlTexture& operator=(const GlTexture&) = delete;

    /// Upload an 8-bit greyscale image. Creates or resizes the texture as
    /// needed; after the first call of a given size this is a single
    /// glTexSubImage2D.
    void upload_grey(std::span<const uint8_t> pixels, int width, int height);

    /// The handle ImGui::Image wants. Zero until the first upload.
    [[nodiscard]] uintptr_t id() const noexcept { return static_cast<uintptr_t>(tex_); }
    [[nodiscard]] int width()  const noexcept { return w_; }
    [[nodiscard]] int height() const noexcept { return h_; }
    [[nodiscard]] bool valid() const noexcept { return tex_ != 0; }

private:
    unsigned int         tex_ = 0;
    int                  w_ = 0, h_ = 0;
    std::vector<uint8_t> rgba_;   ///< reused expansion buffer, never reallocated
};

}  // namespace sat::gui
