// engine/truth_csv.cpp

#include "engine/truth_csv.hpp"

#include <charconv>
#include <fstream>
#include <sstream>
#include <string>

namespace sat {

namespace {

std::vector<std::string> split(const std::string& line) {
    std::vector<std::string> out;
    std::string cur;
    std::istringstream in(line);
    while (std::getline(in, cur, ',')) {
        // Trim, because a human-written CSV has spaces after commas and a
        // parser that rejects them is a parser people work around.
        size_t b = cur.find_first_not_of(" \t\r");
        size_t e = cur.find_last_not_of(" \t\r");
        out.push_back(b == std::string::npos ? std::string() : cur.substr(b, e - b + 1));
    }
    return out;
}

int find_column(const std::vector<std::string>& header, std::initializer_list<const char*> names) {
    for (size_t i = 0; i < header.size(); ++i) {
        for (const char* n : names) {
            if (header[i] == n) return static_cast<int>(i);
        }
    }
    return -1;
}

}  // namespace

Result<std::vector<FrameTruth>>
parse_truth_csv(std::string_view text, const ScreenGeometry& screen,
                std::string_view name) {
    std::istringstream in{std::string(text)};
    std::string line;
    std::vector<std::string> header;
    int ln = 0;

    // The header may be preceded by '#' comment lines — §13.2's own format
    // starts with four of them, and the whole point is that a centroid.csv can
    // be fed back in as truth.
    while (std::getline(in, line)) {
        ++ln;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty()) continue;
        if (line[0] == '#') {
            // §13.2 writes the column list as "# columns: a,b,c". Accept that
            // as the header, so a log file works as input unmodified.
            const size_t at = line.find("columns:");
            if (at != std::string::npos) {
                header = split(line.substr(at + 8));
                break;
            }
            continue;
        }
        header = split(line);
        break;
    }
    if (header.empty()) {
        return Err(std::string(name) + ": no header row found");
    }

    const int c_frame = find_column(header, {"frame", "frame_index", "index"});
    const int c_time  = find_column(header, {"time_s", "time", "t"});
    const int c_x     = find_column(header, {"cx_screen", "x_screen", "screen_x", "x"});
    const int c_y     = find_column(header, {"cy_screen", "y_screen", "screen_y", "y"});
    if (c_x < 0 || c_y < 0) {
        return Err(std::string(name) +
                   ": the header must name screen coordinates (cx_screen, cy_screen); found: " +
                   [&] { std::string s; for (auto& h : header) { s += h; s += ' '; } return s; }());
    }

    std::vector<FrameTruth> out;
    int64_t implicit_index = 0;

    while (std::getline(in, line)) {
        ++ln;
        if (!line.empty() && line.back() == '\r') line.pop_back();
        if (line.empty() || line[0] == '#') continue;
        const std::vector<std::string> f = split(line);

        const int64_t idx = (c_frame >= 0 && c_frame < static_cast<int>(f.size())
                             && !f[static_cast<size_t>(c_frame)].empty())
            ? std::stoll(f[static_cast<size_t>(c_frame)])
            : implicit_index;
        ++implicit_index;

        if (idx < 0) continue;
        if (static_cast<size_t>(idx) >= out.size()) out.resize(static_cast<size_t>(idx) + 1);

        // An empty coordinate means "no truth for this frame". §13.2 writes
        // exactly that on a no-detection row, and a truth file inherits the
        // meaning: the frame is left with n = 0 rather than filled with a
        // guess. Interpolating here would silently invent a reference and every
        // error measured against it would be fiction.
        if (c_x >= static_cast<int>(f.size()) || c_y >= static_cast<int>(f.size())) continue;
        if (f[static_cast<size_t>(c_x)].empty() || f[static_cast<size_t>(c_y)].empty()) continue;

        FrameTruth& t = out[static_cast<size_t>(idx)];
        FrameTruth::Target& tgt = t.targets[0];
        tgt.id         = 1;
        tgt.is_primary = true;
        tgt.screen_pos = Pixel2{std::stod(f[static_cast<size_t>(c_x)]),
                                std::stod(f[static_cast<size_t>(c_y)])};
        tgt.world_ang  = screen.to_angle(tgt.screen_pos);
        t.n            = 1;
        t.tick         = idx;
        if (c_time >= 0 && c_time < static_cast<int>(f.size())
            && !f[static_cast<size_t>(c_time)].empty()) {
            // Kept for reference; the source uses the container timestamp.
            (void)std::stod(f[static_cast<size_t>(c_time)]);
        }
    }

    if (out.empty()) return Err(std::string(name) + ": no data rows");
    return Ok(std::move(out));
}

Result<std::vector<FrameTruth>>
load_truth_csv(const std::filesystem::path& path, const ScreenGeometry& screen) {
    std::ifstream in(path, std::ios::binary);
    if (!in) return Err("cannot read truth CSV '" + path.string() + "'");
    const std::string text((std::istreambuf_iterator<char>(in)),
                            std::istreambuf_iterator<char>());
    return parse_truth_csv(text, screen, path.string());
}

}  // namespace sat
