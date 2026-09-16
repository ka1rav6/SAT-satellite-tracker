// core/strategy.cpp

#include "core/strategy.hpp"

namespace sat {

const char* detector_kind_name(DetectorKind k) noexcept {
    switch (k) {
        case DetectorKind::Classical:      return "classical";
        case DetectorKind::BrightestPixel: return "brightest-pixel";
        case DetectorKind::MlAssisted:     return "ml-assisted";
    }
    return "?";
}

const char* filter_kind_name(FilterKind k) noexcept {
    switch (k) {
        case FilterKind::Cv:  return "CV";
        case FilterKind::Imm: return "IMM";
    }
    return "?";
}

const char* predictor_kind_name(PredictorKind k) noexcept {
    switch (k) {
        case PredictorKind::None:  return "none";
        case PredictorKind::Smith: return "smith";
    }
    return "?";
}

}  // namespace sat
