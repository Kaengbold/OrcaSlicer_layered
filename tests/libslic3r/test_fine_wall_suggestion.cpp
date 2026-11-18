#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "libslic3r/FineWallSuggestion.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/TriangleMesh.hpp"

#include <cmath>
#include <cstddef>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;

namespace {
// How many facets of `its` the rules select whose normal points along `axis` (0 x, 1 y, 2 z) with `sign`.
size_t selected_facing(const indexed_triangle_set &its, const FineWallSuggestion &suggestion, int axis, double sign)
{
    size_t count = 0;
    for (size_t idx = 0; idx < its.indices.size(); ++ idx) {
        const Vec3f n = its_face_normal(its, int(idx));
        if (suggestion.selected[idx] && n[axis] * sign > 0.9f)
            ++ count;
    }
    return count;
}
} // namespace

TEST_CASE("The step of a layer grows as a face leans over", "[FineWallSuggestion]")
{
    constexpr double layer_height = 0.2;
    // An upright face leaves no step, a face at 45 degrees one of the layer height.
    CHECK_THAT(fine_wall_step(0., layer_height), WithinAbs(0., 1e-9));
    CHECK_THAT(fine_wall_step(std::sqrt(0.5), layer_height), WithinAbs(layer_height, 1e-9));
    CHECK_THAT(fine_wall_step(-std::sqrt(0.5), layer_height), WithinAbs(layer_height, 1e-9));
    // A flat top or bottom is a surface of its own, not a wall.
    CHECK_THAT(fine_wall_step(1., layer_height), WithinAbs(0., 1e-9));
}

TEST_CASE("Outward walls selects the sides of a cube but not its top and bottom", "[FineWallSuggestion]")
{
    const TriangleMesh mesh = make_cube(10., 10., 10.);
    FineWallRules      rules;
    rules.mode = FineWallMode::OutwardWalls;
    const FineWallSuggestion suggestion = suggest_fine_walls(mesh.its, Transform3d::Identity(), rules);
    REQUIRE(suggestion.selected.size() == mesh.its.indices.size());
    // Two facets per side.
    CHECK(selected_facing(mesh.its, suggestion, 0, 1.) == 2);
    CHECK(selected_facing(mesh.its, suggestion, 0, -1.) == 2);
    CHECK(selected_facing(mesh.its, suggestion, 1, 1.) == 2);
    CHECK(selected_facing(mesh.its, suggestion, 1, -1.) == 2);
    CHECK(selected_facing(mesh.its, suggestion, 2, 1.) == 0);
    CHECK(selected_facing(mesh.its, suggestion, 2, -1.) == 0);
}

TEST_CASE("Slope mode leaves the upright sides of a cube alone", "[FineWallSuggestion]")
{
    const TriangleMesh mesh = make_cube(10., 10., 10.);
    FineWallRules      rules;
    rules.mode     = FineWallMode::Slope;
    rules.min_step = 0.1;
    const FineWallSuggestion suggestion = suggest_fine_walls(mesh.its, Transform3d::Identity(), rules);
    for (size_t idx = 0; idx < suggestion.selected.size(); ++ idx)
        CHECK_FALSE(suggestion.selected[idx]);
}

TEST_CASE("Slope mode selects the sides of a pyramid", "[FineWallSuggestion]")
{
    // A pyramid 20 mm wide and 10 mm high: its sides lean 45 degrees and step by the layer height.
    const indexed_triangle_set its { { {0, 1, 2}, {0, 3, 4}, {3, 1, 4}, {1, 3, 2}, {3, 0, 2}, {4, 1, 0} },
                                     { {10.f, 10.f, 10.f}, {0.f, 0.f, 0.f}, {20.f, 0.f, 0.f}, {20.f, 20.f, 0.f}, {0.f, 20.f, 0.f} } };
    FineWallRules rules;
    rules.mode         = FineWallMode::Slope;
    rules.layer_height = 0.2;
    rules.min_step     = 0.1;
    const FineWallSuggestion suggestion = suggest_fine_walls(its, Transform3d::Identity(), rules);
    size_t selected = 0;
    for (size_t idx = 0; idx < suggestion.selected.size(); ++ idx)
        if (suggestion.selected[idx]) {
            ++ selected;
            CHECK(suggestion.score[idx] > 0.f);
        }
    // The four sides, not the bottom.
    CHECK(selected == 4);
}

TEST_CASE("The walls of a closed cavity are left out as hidden", "[FineWallSuggestion]")
{
    // A 10 mm cube with a closed 4 mm cavity in its middle, whose faces point into the cavity.
    TriangleMesh mesh   = make_cube(10., 10., 10.);
    TriangleMesh cavity = make_cube(4., 4., 4.);
    cavity.translate(3.f, 3.f, 3.f);
    its_flip_triangles(cavity.its);
    mesh.merge(cavity);
    FineWallRules rules;
    rules.mode         = FineWallMode::OutwardWalls;
    rules.visible_only = false;
    const FineWallSuggestion all = suggest_fine_walls(mesh.its, Transform3d::Identity(), rules);
    rules.visible_only = true;
    const FineWallSuggestion visible = suggest_fine_walls(mesh.its, Transform3d::Identity(), rules);
    size_t num_all = 0;
    size_t num_visible = 0;
    for (size_t idx = 0; idx < mesh.its.indices.size(); ++ idx) {
        num_all     += all.selected[idx] ? 1 : 0;
        num_visible += visible.selected[idx] ? 1 : 0;
    }
    // Two facets on each of the four sides, outside and in the cavity.
    CHECK(num_all == 16);
    CHECK(num_visible == 8);
}
