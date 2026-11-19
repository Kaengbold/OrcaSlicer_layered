#include <catch2/catch_test_macros.hpp>
#include <catch2/generators/catch_generators.hpp>
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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdlib>
#include <initializer_list>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <utility>
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
    // Walls and infill printed at the layer height.
    std::vector<const ExtrusionPath*> at_print_z;
    // Passes, printed under print_z and the top one at it.
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

// The volume a path extrudes in the G-code, where a sloped path scales its flow from its start to its end ratio, and a
// Z contoured one by its height over that of the path, (height + z) / height.
double extruded_volume(const ExtrusionPath &path)
{
    if (path.z_contoured) {
        double volume = 0.;
        for (size_t idx = 1; idx < path.polyline.points.size(); ++ idx) {
            const Point3 &a = path.polyline.points[idx - 1];
            const Point3 &b = path.polyline.points[idx];
            const double  z = unscale<double>(0.5 * double(a.z() + b.z()));
            volume += path.mm3_per_mm * unscale<double>((b - a).head<2>().cast<double>().norm()) * (path.height + z) / path.height;
        }
        return volume;
    }
    const auto *sloped = dynamic_cast<const ExtrusionPathSloped*>(&path);
    return sloped == nullptr ? path.total_volume() : path.total_volume() * 0.5 * (sloped->slope_begin.e_ratio + sloped->slope_end.e_ratio);
}

double volume_of(const Print &print)
{
    double volume = 0.;
    for (const Layer *layer : print.objects().front()->layers()) {
        const LayerPaths paths = paths_of(*layer);
        for (const ExtrusionPath *path : paths.at_print_z)
            volume += extruded_volume(*path);
        for (const ExtrusionPath *path : paths.under)
            volume += extruded_volume(*path);
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

    // The top pass at print_z, the others under it, each a pass height lower.
    REQUIRE(paths.pass_z_offsets.size() == size_t(pass_count));
    for (int pass = 0; pass < pass_count; ++ pass)
        CHECK_THAT(paths.pass_z_offsets[pass], WithinAbs(- (pass_count - 1 - pass) * pass_height, 1e-6));

    // The cube has a single outline: one fine outer wall per pass, and none at the layer height.
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) == size_t(pass_count));
    CHECK(count_paths(paths.at_print_z, erExternalPerimeter, pass_height) == 0);
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
    CHECK(paths.pass_z_offsets.size() == size_t(pass_count));
    CHECK(count_paths(paths.under, erExternalPerimeter, pass_height) == size_t(pass_count));
    // Nothing of the layer is printed at its own height.
    CHECK(paths.at_print_z.empty());
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

TEST_CASE("A scarf joint ramps the wall of the layer over a wedge of the passes under it", "[LayerRefinement]")
{
    Print print;
    slice_one_side_refined(print, pass_height, 3.);
    const LayerPaths paths = paths_of(middle_layer(print));
    // The top line rises from the height of a pass to that of the layer over the ramp.
    size_t ramped = 0;
    for (const ExtrusionPath *path : paths.at_print_z)
        if (path->role() == erExternalPerimeter && path->height > pass_height + 1e-3 && path->height < layer_height - 1e-3)
            ++ ramped;
    CHECK(ramped > 0);
    // The lower passes run on lowered, as a wedge.
    size_t lowered = 0;
    for (const ExtrusionPath *path : paths.under)
        if (path->z_contoured)
            ++ lowered;
    CHECK(lowered > 0);
}

TEST_CASE("The top line of a layer runs through the fine and the coarse wall in one", "[LayerRefinement]")
{
    Print print;
    slice_one_side_refined(print, pass_height, 3.);
    const Layer &layer = middle_layer(print);
    // The outer wall of the layer is one loop holding both the top pass and the wall at the layer height.
    size_t mixed_loops = 0;
    for (const LayerRegion *region : layer.regions()) {
        std::vector<const ExtrusionEntity*> stack(region->perimeters.entities.begin(), region->perimeters.entities.end());
        while (! stack.empty()) {
            const ExtrusionEntity *entity = stack.back();
            stack.pop_back();
            if (const auto *collection = dynamic_cast<const ExtrusionEntityCollection*>(entity))
                stack.insert(stack.end(), collection->entities.begin(), collection->entities.end());
            else if (const auto *loop = dynamic_cast<const ExtrusionLoop*>(entity)) {
                bool fine = false, coarse = false;
                for (const ExtrusionPath &path : loop->paths) {
                    fine   = fine || std::abs(path.height - pass_height) < 1e-4;
                    coarse = coarse || std::abs(path.height - layer_height) < 1e-4;
                }
                if (fine && coarse)
                    ++ mixed_loops;
            }
        }
    }
    CHECK(mixed_loops == 1);
}

TEST_CASE("The top pass spliced into the wall of an unrefined region prints as a fine wall", "[LayerRefinement]")
{
    Print print;
    slice_one_side_refined(print, pass_height, 3.);
    const std::string gcode = Test::gcode(print);
    // The region the top pass is spliced into has no fine wall height of its own. Its wall at the height of a pass is
    // still a fine wall, with the speed, fan and overhang judgement of the passes under it.
    std::istringstream lines(gcode);
    std::string        role;
    double             height = 0.;
    size_t             fine = 0, outer_at_pass_height = 0;
    for (std::string line; std::getline(lines, line);) {
        if (line.rfind(";TYPE:", 0) == 0)
            role = line.substr(6);
        else if (line.rfind(";HEIGHT:", 0) == 0)
            height = std::atof(line.c_str() + 8);
        else if (line.rfind("G1 ", 0) == 0 && line.find(" E") != std::string::npos && line.find(" X") != std::string::npos &&
                 std::abs(height - pass_height) < 1e-3) {
            if (role == "Fine wall")
                ++ fine;
            else if (role == "Outer wall")
                ++ outer_at_pass_height;
        }
    }
    CHECK(fine > 0);
    CHECK(outer_at_pass_height == 0);
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

TEST_CASE("A minimum step leaves upright walls at the layer height", "[LayerRefinement]")
{
    Print print;
    slice(print, Test::cube(20), pass_height, {{"layer_refinement_min_step", "0.1"}});
    for (const Layer *layer : print.objects().front()->layers())
        CHECK(paths_of(*layer).under.empty());
}

TEST_CASE("A minimum step still refines a slope that steps further than it", "[LayerRefinement]")
{
    // The flat pyramid of the slope test: the outlines of the passes of a layer spread 0.75 mm apart.
    Print print;
    slice(print, TriangleMesh({ {10, 10, 2}, {0, 0, 0}, {20, 0, 0}, {20, 20, 0}, {0, 20, 0} },
                              { {0, 1, 2}, {0, 3, 4}, {3, 1, 4}, {1, 3, 2}, {3, 0, 2}, {4, 1, 0} }),
          pass_height, {{"layer_refinement_min_step", "0.1"}});
    CHECK(count_paths(paths_of(middle_layer(print)).under, erExternalPerimeter, pass_height) >= size_t(pass_count - 1));
}

TEST_CASE("A helical fine wall prints the passes of a loop as one rising helix", "[LayerRefinement]")
{
    Print print;
    slice(print, Test::cube(20), pass_height, {{"layer_refinement_helical", "1"}});
    const Layer &layer = middle_layer(print);
    size_t       helices = 0;
    for (const LayerRegion *region : layer.regions())
        for (const LayerRegion::RefinementPass &pass : region->refinement_passes) {
            if (pass.turn_offsets.empty())
                continue;
            ++ helices;
            // A flat turn, a turn per further pass and a flat closing turn, the last one at print_z.
            REQUIRE(pass.turn_offsets.size() == size_t(pass_count + 1));
            CHECK_THAT(pass.turn_offsets.front(), WithinAbs(- (pass_count - 1) * pass_height, 1e-6));
            CHECK_THAT(pass.turn_offsets.back(), WithinAbs(0., 1e-6));
            CHECK(dynamic_cast<const ExtrusionPathSloped*>(pass.extrusions.entities[1]) != nullptr);
        }
    CHECK(helices == 1);
    // The helix ends at print_z, so no fine outer wall is left among the walls of the layer.
    CHECK(count_paths(paths_of(layer).at_print_z, erExternalPerimeter, pass_height) == 0);
}

TEST_CASE("A helical fine wall extrudes the same volume", "[LayerRefinement]")
{
    Print coarse;
    slice(coarse, Test::cube(20), 0.);
    Print helical;
    slice(helical, Test::cube(20), pass_height, {{"layer_refinement_helical", "1"}});
    // The ramps of the first and the closing turn make up for the extra turn.
    CHECK_THAT(volume_of(helical), WithinRel(volume_of(coarse), 0.02));
}

// The G-code of an object of two 10 mm cubes 10 mm apart, refined, with further settings.
static std::string two_island_gcode(std::initializer_list<ConfigBase::SetDeserializeItem> more)
{
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"wall_loops", "2"},
        {"layer_refinement_height", std::to_string(pass_height)},
    });
    config.set_deserialize_strict(more);
    TriangleMesh mesh = make_cube(10., 10., 10.);
    TriangleMesh other = make_cube(10., 10., 10.);
    other.translate(20.f, 0.f, 0.f);
    mesh.merge(other);
    return Test::slice({std::move(mesh)}, config);
}

// How often an extrusion is printed lower than the one before it.
static size_t extrusion_z_drops(const std::string &gcode)
{
    std::istringstream lines(gcode);
    double z = 0., last_z = 0.;
    size_t drops = 0;
    for (std::string line; std::getline(lines, line);) {
        if (line.rfind("G1 ", 0) != 0 && line.rfind("G0 ", 0) != 0)
            continue;
        const std::string move = line.substr(0, line.find(';'));
        if (const size_t pos = move.find(" Z"); pos != std::string::npos)
            z = std::atof(move.c_str() + pos + 2);
        const size_t e = move.find(" E");
        if (e == std::string::npos || std::atof(move.c_str() + e + 2) <= 0. || move.find(" X") == std::string::npos)
            continue;
        if (z < last_z - 1e-4)
            ++ drops;
        last_z = z;
    }
    return drops;
}

TEST_CASE("Chained passes print the passes of one island before the next", "[LayerRefinement]")
{
    // The passes go up through the layer: printed pass by pass over both islands they never go down, island by
    // island they go down once on each refined layer, to the lowest pass of the second island.
    CHECK(extrusion_z_drops(two_island_gcode({{"layer_refinement_chain_passes", "0"}})) == 0);
    CHECK(extrusion_z_drops(two_island_gcode({{"layer_refinement_chain_passes", "1"}})) > 0);
}

TEST_CASE("A fine wall acceleration is set for the passes", "[LayerRefinement]")
{
    const auto gcode_with = [](const char *acceleration) {
        DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
        config.set_deserialize_strict({
            {"layer_height", std::to_string(layer_height)},
            {"initial_layer_print_height", std::to_string(layer_height)},
            {"gcode_flavor", "marlin2"},
            {"default_acceleration", "1000"},
            {"outer_wall_acceleration", "1000"},
            {"layer_refinement_height", std::to_string(pass_height)},
            {"layer_refinement_acceleration", acceleration},
        });
        return Test::slice({Test::cube(20)}, config);
    };
    // Marlin 2 sets the print acceleration with M204 P. A value under the machine limits is not clamped.
    CHECK(gcode_with("777").find("M204 P777") != std::string::npos);
    CHECK(gcode_with("0").find("M204 P777") == std::string::npos);
}

TEST_CASE("Inner walls keep the layer height where only one side of a layer moves inwards", "[LayerRefinement]")
{
    // A 20 mm box whose side at x = 20 leans out by 2 mm per mm of height. The passes of a layer on that side lie
    // further in the lower they are, so their outer walls run over the inner wall of the layer there, and only there.
    Print print;
    slice(print, TriangleMesh({ {0, 0, 0}, {20, 0, 0}, {20, 20, 0}, {0, 20, 0}, {0, 0, 5}, {30, 0, 5}, {30, 20, 5}, {0, 20, 5} },
                              { {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                                {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7} }),
          pass_height);
    const LayerPaths paths = paths_of(middle_layer(print));
    double inner_wall_length = 0.;
    for (const ExtrusionPath *path : paths.at_print_z)
        if (path->role() == erPerimeter && std::abs(path->height - layer_height) < 1e-4)
            inner_wall_length += unscale<double>(path->length());
    // The three upright sides, about 19 mm each, keep their inner wall at the layer height.
    CHECK(inner_wall_length > 50.);
}

TEST_CASE("A short travel from a fine wall goes without a retraction", "[LayerRefinement]")
{
    // Two cubes 10 mm apart, printed pass by pass over both: every pass travels 10 mm from one cube to the other.
    const auto retractions = [](const char *distance) {
        const std::string  gcode = two_island_gcode({{"layer_refinement_chain_passes", "0"},
                                                     {"layer_refinement_travel_without_retraction", distance}});
        size_t             count = 0;
        std::istringstream lines(gcode);
        for (std::string line; std::getline(lines, line);)
            if (line.rfind("G1 E-", 0) == 0)
                ++ count;
        return count;
    };
    CHECK(retractions("50") < retractions("0"));
}

TEST_CASE("A refined layer keeps its top surfaces ironed", "[LayerRefinement]")
{
    // A 20 mm block 10 mm high with a 10 mm tower on it: the layer at 10 mm has a top surface around the tower and an
    // inner area under it, which must keep their own types for the ironing to find the top.
    TriangleMesh mesh  = make_cube(20., 20., 10.);
    TriangleMesh tower = make_cube(10., 10., 20.);
    tower.translate(5.f, 5.f, 0.f);
    mesh.merge(tower);
    Print print;
    slice(print, std::move(mesh), pass_height, {{"ironing_type", "top"}});
    size_t ironing = 0;
    for (const Layer *layer : print.objects().front()->layers())
        if (std::abs(layer->print_z - 10.) < 0.01)
            for (const ExtrusionPath *path : paths_of(*layer).at_print_z)
                if (path->role() == erIroning)
                    ++ ironing;
    CHECK(ironing > 0);
}

TEST_CASE("All passes of a layer get the same speed on an overhang", "[LayerRefinement]")
{
    // A 20 mm box leaning out by 2 mm per mm on one side, refined whole, with overhang speeds and no slowdown for the
    // layer time. Every line of a pass, and every turn of a helix, is judged as the wall of the layer it replaces,
    // whatever its own width, so the passes of a layer print the upright and the leaning side of the box at the same
    // speeds. Only the corners differ: there the higher passes of the leaning side reach further out than the lower ones.
    const std::string helical = GENERATE("0", "1");
    DynamicPrintConfig config = DynamicPrintConfig::full_print_config();
    config.set_deserialize_strict({
        {"layer_height", std::to_string(layer_height)},
        {"initial_layer_print_height", std::to_string(layer_height)},
        {"wall_loops", "2"},
        {"enable_overhang_speed", "1"},
        {"slow_down_for_layer_cooling", "0"},
        {"layer_refinement_height", std::to_string(pass_height)},
        {"layer_refinement_helical", helical},
    });
    const std::string gcode = Test::slice({TriangleMesh({ {0, 0, 0}, {20, 0, 0}, {20, 20, 0}, {0, 20, 0}, {0, 0, 5}, {30, 0, 5}, {30, 20, 5}, {0, 20, 5} },
                                                        { {0, 2, 1}, {0, 3, 2}, {4, 5, 6}, {4, 6, 7}, {0, 1, 5}, {0, 5, 4}, {1, 2, 6}, {1, 6, 5},
                                                          {2, 3, 7}, {2, 7, 6}, {3, 0, 4}, {3, 4, 7} })},
                                          config);
    // The extrusions of the fine walls along the sides: layer, height of the pass or turn, x, y from and to, speed.
    struct Move
    {
        std::string layer, z;
        double      x, from_y, to_y;
        long        feedrate;
    };
    std::vector<Move>  moves;
    std::istringstream lines(gcode);
    std::string        layer, z, role;
    long               feedrate = 0;
    double             x = 0., y = 0.;
    const auto         value = [](const std::string &move, const char *axis, double &out) {
        if (const size_t pos = move.find(axis); pos != std::string::npos)
            out = std::atof(move.c_str() + pos + 2);
    };
    for (std::string line; std::getline(lines, line);) {
        if (line.rfind(";Z:", 0) == 0)
            layer = line.substr(3);
        else if (line.rfind(";TYPE:", 0) == 0)
            role = line.substr(6);
        else if (line.rfind("G1 ", 0) == 0) {
            const std::string move = line.substr(0, line.find(';'));
            if (const size_t pos = move.find(" F"); pos != std::string::npos)
                feedrate = std::lround(std::atof(move.c_str() + pos + 2));
            if (const size_t pos = move.find(" Z"); pos != std::string::npos)
                z = move.substr(pos + 2, move.find(' ', pos + 1) - pos - 2);
            const double from_x = x, from_y = y;
            double       e      = 0.;
            value(move, " X", x);
            value(move, " Y", y);
            value(move, " E", e);
            if (role == "Fine wall" && e > 0. && std::abs(x - from_x) < 0.1 && std::abs(y - from_y) > 0.1)
                moves.push_back({layer, z, x, from_y, y, feedrate});
        }
    }
    REQUIRE(! moves.empty());
    // The box is 20 mm deep wherever it is placed: compare the stretch of the sides 2 mm or more from the corners.
    double min_x = moves.front().x, max_x = min_x, min_y = moves.front().from_y;
    for (const Move &move : moves) {
        min_x = std::min(min_x, move.x);
        max_x = std::max(max_x, move.x);
        min_y = std::min({min_y, move.from_y, move.to_y});
    }
    // Layer and side of the box, upright or leaning -> height of the pass or turn -> the speeds it prints at.
    std::map<std::pair<std::string, bool>, std::map<std::string, std::set<long>>> speeds;
    for (const Move &move : moves)
        if (std::max(move.from_y, move.to_y) > min_y + 2. && std::min(move.from_y, move.to_y) < min_y + 18.)
            speeds[{move.layer, move.x > 0.5 * (min_x + max_x)}][move.z].insert(move.feedrate);
    size_t compared = 0;
    for (const auto &[layer_side, passes] : speeds) {
        if (passes.size() < 2)
            continue;
        for (const auto &[pass_z, pass_speeds] : passes)
            CHECK(pass_speeds == passes.begin()->second);
        ++ compared;
    }
    CHECK(compared > 0);
}
