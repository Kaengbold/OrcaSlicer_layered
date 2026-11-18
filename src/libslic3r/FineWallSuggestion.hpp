#pragma once

#include "Point.hpp"

#include <cstdint>
#include <vector>

#include <admesh/stl.h>

namespace Slic3r {

// Orca: Which faces of a part should get fine walls, see the "Suggestions" section of docs/HLSD/layer-refinement.md.
enum class FineWallMode : uint8_t
{
    // Nothing is suggested, the user paints.
    Paint,
    // Every wall that can be seen from outside.
    OutwardWalls,
    // The walls where a layer leaves a visible step: slopes, domes and, optionally, overhangs.
    Slope,
};

struct FineWallRules
{
    FineWallMode mode           { FineWallMode::Slope };
    // Layer height of the print, which sets the size of the steps.
    double       layer_height   { 0.2 };
    // Slope mode: the smallest horizontal step a layer leaves on a face for it to be refined, in mm.
    double       min_step       { 0.1 };
    // Slope mode: also faces overhanging by at least this angle from upright, in degrees.
    bool         overhangs      { true };
    double       overhang_angle { 45. };
    // Leave out faces that cannot be seen from outside, as inside holes and cavities.
    bool         visible_only   { true };
};

struct FineWallSuggestion
{
    // Per facet: how much a fine wall gains there, from 0 (an upright wall or a flat top) to 1 (a step of half a
    // millimetre or more).
    std::vector<float> score;
    // Per facet: whether the rules choose it.
    std::vector<bool>  selected;
};

// The horizontal step a layer of `layer_height` leaves on a face whose unit normal points `normal_z` upwards.
// Flat tops and bottoms, within half a degree, are printed as top and bottom surfaces and leave none.
double fine_wall_step(double normal_z, double layer_height);

// Scores and selects the facets of a part. `trafo` places the part in the world, so that the steps are measured
// against the print's Z axis.
FineWallSuggestion suggest_fine_walls(const indexed_triangle_set &its, const Transform3d &trafo, const FineWallRules &rules);

} // namespace Slic3r
