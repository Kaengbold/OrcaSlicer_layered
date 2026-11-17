#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include "test_helpers.hpp"

#include "libslic3r/ExtrusionEntity.hpp"
#include "libslic3r/ExtrusionEntityCollection.hpp"
#include "libslic3r/Layer.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Print.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleMesh.hpp"
#include "libslic3r/libslic3r.h"

#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <sstream>
#include <string>
#include <vector>

using namespace Slic3r;
using Catch::Matchers::WithinAbs;
using Catch::Matchers::WithinRel;

namespace {
constexpr double layer_height = 0.2;
constexpr double pass_height  = 0.05;
constexpr int    pass_count   = 4;

// Slices one mesh at 0.2 mm with 2 walls, with the given fine wall layer height and further settings.
void slice(Print &print, TriangleMesh mesh, double refinement_height, std::initializer_list<ConfigBase::SetDeserializeItem> more = {})
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"wall_loops", "2"},
        {"sparse_infill_density", "15%"},
        {"layer_refinement_height", std::to_string(refinement_height)},
    });
    config.set_deserialize_strict(more);
    Test::init_and_process_print({std::move(mesh)}, print, config);
}

// The single paths of an extrusion collection, loops and nested collections taken apart.
void collect_paths(const ExtrusionEntityCollection &collection, std::vector<const ExtrusionPath*> &out)
{
    for (const ExtrusionEntity *entity : collection.entities)
        if (const auto *child = dynamic_cast<const ExtrusionEntityCollection*>(entity))
            collect_paths(*child, out);
        else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity))
            for (const ExtrusionPath &path : loop->paths)
                out.emplace_back(&path);
        else if (const auto *multi_path = dynamic_cast<const ExtrusionMultiPath*>(entity))
            for (const ExtrusionPath &path : multi_path->paths)
                out.emplace_back(&path);
        else if (const auto *path = dynamic_cast<const ExtrusionPath*>(entity))
            out.emplace_back(path);
}

struct LayerPaths
{
    // Walls and infill printed at print_z.
    std::vector<const ExtrusionPath*> at_print_z;
    // Passes printed under print_z.
    std::vector<const ExtrusionPath*> under;
    std::vector<double>               pass_z_offsets;
};

LayerPaths paths_of(const Layer &layer)
{
    LayerPaths out;
    for (const LayerRegion *region : layer.regions()) {
        collect_paths(region->perimeters, out.at_print_z);
        collect_paths(region->fills, out.at_print_z);
        for (const LayerRegion::RefinementPass &pass : region->refinement_passes) {
            collect_paths(pass.extrusions, out.under);
            out.pass_z_offsets.emplace_back(pass.z_offset);
        }
    }
    return out;
}

size_t count_paths(const std::vector<const ExtrusionPath*> &paths, ExtrusionRole role, double height)
{
    size_t count = 0;
    for (const ExtrusionPath *path : paths)
        if (path->role() == role && std::abs(path->height - height) < 1e-4)
            ++ count;
    return count;
}

double volume_of(const Print &print)
{
    double volume = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        const LayerPaths paths = paths_of(*layer);
        for (const ExtrusionPath *path : paths.at_print_z)
            volume += path->total_volume();
        for (const ExtrusionPath *path : paths.under)
            volume += path->total_volume();
    }
    return volume;
}

const Layer &middle_layer(const Print &print)
{
    const auto &layers = print.objects().front()->layers();
    REQUIRE(layers.size() > 4);
    return *layers[layers.size() / 2];
}
} // namespace

TEST_CASE("A fine wall layer height is snapped to a whole number of passes per layer", "[LayerRefinement]")
{
    CHECK(layer_refinement_pass_count(0.2, 0.) == 1);
    CHECK(layer_refinement_pass_count(0.2, 0.05) == 4);
    CHECK(layer_refinement_pass_count(0.2, 0.06) == 3);
    CHECK(layer_refinement_pass_count(0.2, 0.1) == 2);
    // A pass as high as the layer, or higher, is the layer itself.
    CHECK(layer_refinement_pass_count(0.2, 0.2) == 1);
    CHECK(layer_refinement_pass_count(0.2, 0.3) == 1);
    // Passes are never thinner than 0.02 mm.
    CHECK(layer_refinement_pass_count(0.2, 0.001) == 10);
}

TEST_CASE("Layers are not refined without a fine wall layer height", "[LayerRefinement]")
{
    Print print;
    slice(print, Test::cube(20), 0.);
    for (const Layer *layer : print.objects().front()->layers()) {
        const LayerPaths paths = paths_of(*layer);
        CHECK(paths.under.empty());
        CHECK(count_paths(paths.at_print_z, erExternalPerimeter, pass_height) == 0);
    }
}

TEST_CASE("An upright wall is refined into fine outer walls in front of the walls of the layer", "[LayerRefinement]")
{
    Print print;
    slice(print, Test::cube(20), pass_height);
    const LayerPaths paths = paths_of(middle_layer(print));

    // One pass at print_z, the others under it, each a pass height lower.
    REQUIRE(paths.pass_z_offsets.size() == size_t(pass_count - 1));
    for (int pass = 0; pass < pass_count - 1; ++ pass)
        CHECK_THAT(paths.pass_z_offsets[pass], WithinAbs(- (pass_count - 1 - pass) * pass_height, 1e-6));

    // The cube has a single outline: one fine outer wall per pass, and none at the layer height.
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) == size_t(pass_count - 1));
    CHECK(count_paths(paths.at_print_z, erExternalPerimeter, pass_height) == 1);
    CHECK(count_paths(paths.at_print_z, erExternalPerimeter, layer_height) == 0);
    // Only the outer wall is replaced: the inner wall and the infill stay at the layer height.
    CHECK(count_paths(paths.under, erPerimeter, pass_height) == 0);
    CHECK(count_paths(paths.at_print_z, erPerimeter, layer_height) == 1);
    CHECK(count_paths(paths.at_print_z, erInternalInfill, layer_height) > 0);
}

TEST_CASE("The first layer is not refined", "[LayerRefinement]")
{
    Print print;
    slice(print, Test::cube(20), pass_height);
    const LayerPaths paths = paths_of(*print.objects().front()->layers().front());
    CHECK(paths.under.empty());
    CHECK(count_paths(paths.at_print_z, erExternalPerimeter, pass_height) == 0);
}

TEST_CASE("Refining an upright wall extrudes the same volume", "[LayerRefinement]")
{
    Print coarse;
    slice(coarse, Test::cube(20), 0.);
    Print refined;
    slice(refined, Test::cube(20), pass_height);
    CHECK_THAT(volume_of(refined), WithinRel(volume_of(coarse), 0.02));
}

TEST_CASE("A feature too thin for a wall behind the fine one is printed by the passes alone", "[LayerRefinement]")
{
    Print print;
    slice(print, make_cube(0.7, 0.7, 5.), pass_height);
    const LayerPaths paths = paths_of(middle_layer(print));
    CHECK(paths.pass_z_offsets.size() == size_t(pass_count - 1));
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) == size_t(pass_count - 1));
    for (const ExtrusionPath *path : paths.at_print_z)
        CHECK_THAT(path->height, WithinAbs(pass_height, 1e-4));
}

TEST_CASE("A slope is refined over more than its outer wall", "[LayerRefinement]")
{
    // A flat pyramid 20 mm wide and 2 mm high narrows by 1 mm per side per layer, so that the outlines of the
    // passes of a layer spread 0.75 mm apart, further than one wall.
    Print print;
    slice(print, TriangleMesh({ {10, 10, 2}, {0, 0, 0}, {20, 0, 0}, {20, 20, 0}, {0, 20, 0} },
                              { {0, 1, 2}, {0, 3, 4}, {3, 1, 4}, {1, 3, 2}, {3, 0, 2}, {4, 1, 0} }), pass_height);
    const LayerPaths paths = paths_of(middle_layer(print));
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) >= size_t(pass_count - 1));
    CHECK(count_paths(paths.under, erPerimeter, pass_height) > 0);
}

// Slices a 20 mm cube with 2 walls at 0.2 mm, with a modifier over its side at x = 0 only that sets the fine wall
// layer height, if any, and the scarf length.
static void slice_one_side_refined(Print &print, double refinement_height, double scarf_length)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"wall_loops", "2"},
        {"sparse_infill_density", "15%"},
        {"layer_refinement_height", "0"},
    });
    Model        model;
    ModelObject *object = model.add_object();
    object->add_volume(make_cube(20., 20., 20.));
    TriangleMesh modifier_mesh = make_cube(10., 30., 30.);
    modifier_mesh.translate(-5.f, -5.f, -5.f);
    ModelVolume *modifier = object->add_volume(std::move(modifier_mesh), ModelVolumeType::PARAMETER_MODIFIER);
    modifier->config.set("layer_refinement_height", refinement_height);
    modifier->config.set("layer_refinement_scarf_length", scarf_length);
    object->add_instance()->set_offset(Vec3d(100., 100., 0.));
    object->ensure_on_bed();
    print.apply(model, config);
    print.process();
}

TEST_CASE("A modifier refines only the walls inside it", "[LayerRefinement]")
{
    Print print;
    slice_one_side_refined(print, pass_height, 0.);

    const Layer &layer           = middle_layer(print);
    size_t       refined_regions = 0;
    for (const LayerRegion *region : layer.regions())
        if (! region->refinement_passes.empty())
            ++ refined_regions;
    CHECK(refined_regions == 1);
    const LayerPaths paths = paths_of(layer);
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) > 0);
    // The other sides keep their outer wall at the layer height.
    CHECK(count_paths(paths.at_print_z, erExternalPerimeter, layer_height) > 0);
    // Without a scarf, the outer wall of the layer is never thinned. A share of one pass height would be the top pass.
    for (int share = 2; share < pass_count; ++ share)
        CHECK(count_paths(paths.at_print_z, erExternalPerimeter, layer_height * share / pass_count) == 0);
}

TEST_CASE("A scarf joint thins the wall of the layer in steps over the passes run on under it", "[LayerRefinement]")
{
    Print print;
    slice_one_side_refined(print, pass_height, 3.);
    const LayerPaths paths = paths_of(middle_layer(print));
    // Each step of the scarf keeps one more pass height of the outer wall of the layer: a quarter, a half, three
    // quarters. The quarter is as high as the top pass, so only the half and three quarters tell the steps apart.
    for (int share = 2; share < pass_count; ++ share)
        CHECK(count_paths(paths.at_print_z, erExternalPerimeter, layer_height * share / pass_count) > 0);
}

TEST_CASE("A scarf joint extrudes the same volume", "[LayerRefinement]")
{
    Print coarse;
    slice_one_side_refined(coarse, 0., 0.);
    Print scarfed;
    slice_one_side_refined(scarfed, pass_height, 3.);
    CHECK_THAT(volume_of(scarfed), WithinRel(volume_of(coarse), 0.02));
}

TEST_CASE("The passes are printed under the layer in the G-code", "[LayerRefinement]")
{
    // Whether the G-code moves to a height a pass height above a layer, which only a pass of a refined layer does.
    const auto moves_to_pass_height = [](const std::string &gcode) {
        std::istringstream lines(gcode);
        for (std::string line; std::getline(lines, line);) {
            if (line.rfind("G1 ", 0) != 0)
                continue;
            const size_t pos = line.find(" Z");
            if (pos == std::string::npos)
                continue;
            const double z = std::atof(line.c_str() + pos + 2);
            if (std::abs(std::fmod(z + 1e-6, layer_height) - pass_height) < 1e-3)
                return true;
        }
        return false;
    };

    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"wall_loops", "2"},
        {"z_hop", "0"},
        {"layer_refinement_height", std::to_string(pass_height)},
    });
    CHECK(moves_to_pass_height(Test::slice({Test::cube(20)}, config)));

    config.set_deserialize_strict({{"layer_refinement_height", "0"}});
    CHECK_FALSE(moves_to_pass_height(Test::slice({Test::cube(20)}, config)));
}
