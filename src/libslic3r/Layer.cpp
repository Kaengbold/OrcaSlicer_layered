#include "Layer.hpp"
#include "AABBTreeLines.hpp"
#include "ClipperUtils.hpp"
#include "Polygon.hpp"
#include "Point.hpp"
#include "ExPolygon.hpp"
#include "ExtrusionEntity.hpp"
#include "ExtrusionEntityCollection.hpp"
#include "Exception.hpp"
#include "Flow.hpp"
#include "Polyline.hpp"
#include "Print.hpp"
#include "PrintConfig.hpp"
#include "ShortestPath.hpp"
#include "SVG.hpp"
#include "BoundingBox.hpp"
#include "Surface.hpp"
#include "SurfaceCollection.hpp"
#include "libslic3r.h"
#include "Utils.hpp"

#include <algorithm>
#include <boost/log/trivial.hpp>
#include <vector>
#include <cstddef>
#include <utility>
#include <cassert>
#include <cmath>
#include <limits>
#include <map>
#include <memory>
#include <numeric>
#include "Config.hpp"
#include "MultiMaterialSegmentation.hpp"
#include "ObjectID.hpp"

namespace Slic3r {

Layer::~Layer()
{
    this->lower_layer = this->upper_layer = nullptr;
    for (LayerRegion *region : m_regions)
        delete region;
    m_regions.clear();
}

// Test whether whether there are any slices assigned to this layer.
bool Layer::empty() const
{
	for (const LayerRegion *layerm : m_regions)
        if (layerm != nullptr && ! layerm->slices.empty())
            // Non empty layer.
            return false;
    return true;
}

LayerRegion* Layer::add_region(const PrintRegion *print_region)
{
    m_regions.emplace_back(new LayerRegion(this, print_region));
    return m_regions.back();
}

// merge all regions' slices to get islands
void Layer::make_slices()
{
    ExPolygons slices;
    if (m_regions.size() == 1) {
        // optimization: if we only have one region, take its slices
        slices = to_expolygons(m_regions.front()->slices.surfaces);
    } else {
        Polygons slices_p;
        for (LayerRegion *layerm : m_regions)
            polygons_append(slices_p, to_polygons(layerm->slices.surfaces));
        slices = union_safety_offset_ex(slices_p);
    }

    this->lslices.clear();
    this->lslices.reserve(slices.size());

    // prepare ordering points
    Points ordering_points;
    ordering_points.reserve(slices.size());
    for (const ExPolygon &ex : slices)
        ordering_points.push_back(ex.contour.first_point());

    // sort slices
    std::vector<Points::size_type> order = chain_points(ordering_points);

    // populate slices vector
    for (size_t i : order)
        this->lslices.emplace_back(std::move(slices[i]));
}

static inline bool layer_needs_raw_backup(const Layer *layer)
{
    // BBS: backup raw slice for generating support
    //return ! (layer->regions().size() == 1 && (layer->id() > 0 || layer->object()->config().elefant_foot_compensation.value == 0));
    return true;
}

void Layer::backup_untyped_slices()
{
    if (layer_needs_raw_backup(this)) {
        for (LayerRegion *layerm : m_regions)
            layerm->raw_slices = to_expolygons(layerm->slices.surfaces);
    } else {
        assert(m_regions.size() == 1);
        m_regions.front()->raw_slices.clear();
    }
}

void Layer::restore_untyped_slices()
{
    if (layer_needs_raw_backup(this)) {
        for (LayerRegion *layerm : m_regions)
            layerm->slices.set(layerm->raw_slices, stInternal);
    } else {
        assert(m_regions.size() == 1);
        m_regions.front()->slices.set(this->lslices, stInternal);
    }
}

// Similar to Layer::restore_untyped_slices()
// To improve robustness of detect_surfaces_type() when reslicing (working with typed slices), see GH issue #7442.
// Only resetting layerm->slices if Slice::extra_perimeters is always zero or it will not be used anymore
// after the perimeter generator.
void Layer::restore_untyped_slices_no_extra_perimeters()
{
    if (layer_needs_raw_backup(this)) {
        for (LayerRegion *layerm : m_regions)
            //BBS: remove extra_perimeters. Always false
        	//if (! layerm->region().config().extra_perimeters.value)
            	layerm->slices.set(layerm->raw_slices, stInternal);
    } else {
    	assert(m_regions.size() == 1);
    	LayerRegion *layerm = m_regions.front();
    	// This optimization is correct, as extra_perimeters are only reused by prepare_infill() with multi-regions.
        //if (! layerm->region().config().extra_perimeters.value)
        	layerm->slices.set(this->lslices, stInternal);
    }
}

ExPolygons Layer::merged(float offset_scaled) const
{
	assert(offset_scaled >= 0.f);
    // If no offset is set, apply EPSILON offset before union, and revert it afterwards.
	float offset_scaled2 = 0;
	if (offset_scaled == 0.f) {
		offset_scaled  = float(  EPSILON);
		offset_scaled2 = float(- EPSILON);
    }
    Polygons polygons;
	for (LayerRegion *layerm : m_regions) {
		const PrintRegionConfig &config = layerm->region().config();
		// Our users learned to bend Slic3r to produce empty volumes to act as subtracters. Only add the region if it is non-empty.
		if (config.bottom_shell_layers > 0 || config.top_shell_layers > 0 || config.sparse_infill_density > 0. || config.wall_loops > 0)
			append(polygons, offset(layerm->slices.surfaces, offset_scaled));
	}
    ExPolygons out = union_ex(polygons);
	if (offset_scaled2 != 0.f)
		out = offset_ex(out, offset_scaled2);
    return out;
}

bool Layer::is_perimeter_compatible(const Print& print, const PrintRegion& a, const PrintRegion& b)
{
    const PrintRegionConfig& config       = a.config();
    const PrintRegionConfig& other_config = b.config();

        return config.outer_wall_filament_id       == other_config.outer_wall_filament_id
		&& config.inner_wall_filament_id       == other_config.inner_wall_filament_id
		&& config.wall_loops                  == other_config.wall_loops
		&& config.wall_sequence               == other_config.wall_sequence
		&& config.is_infill_first             == other_config.is_infill_first
		&& config.inner_wall_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id)) == other_config.inner_wall_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id))
		&& config.outer_wall_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id)) == other_config.outer_wall_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id))
		&& config.small_perimeter_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id)) == other_config.small_perimeter_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id))
		&& config.small_support_perimeter_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id)) == other_config.small_support_perimeter_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id))
        && config.gap_infill_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id)) == other_config.gap_infill_speed.get_at(print.get_extruder_id(config.outer_wall_filament_id))
        && config.filter_out_gap_fill.value == other_config.filter_out_gap_fill.value
		&& config.detect_overhang_wall                   == other_config.detect_overhang_wall
		&& config.unsupported_wall_last                  == other_config.unsupported_wall_last
		&& config.overhang_reverse                       == other_config.overhang_reverse
		&& config.overhang_reverse_threshold             == other_config.overhang_reverse_threshold
		&& config.wall_direction                         == other_config.wall_direction
		&& config.opt_serialize("inner_wall_line_width") == other_config.opt_serialize("inner_wall_line_width")
		&& config.opt_serialize("outer_wall_line_width") == other_config.opt_serialize("outer_wall_line_width")
		&& config.detect_thin_wall                  == other_config.detect_thin_wall
		&& config.infill_wall_overlap              == other_config.infill_wall_overlap
        && config.top_bottom_infill_wall_overlap              == other_config.top_bottom_infill_wall_overlap
        // Orca: these flags directly change the effective wall count produced by the perimeter
        // generator. If two regions disagree on any of them, merging their slices into one shared make_perimeters
        // call would silently use the first region's flag for both.
        && config.only_one_wall_first_layer == other_config.only_one_wall_first_layer
        && config.only_one_wall_top         == other_config.only_one_wall_top
        && config.min_width_top_surface     == other_config.min_width_top_surface
        && config.seam_slope_type         == other_config.seam_slope_type
        && config.seam_slope_conditional == other_config.seam_slope_conditional
        && config.scarf_angle_threshold  == other_config.scarf_angle_threshold
        && config.scarf_overhang_threshold  == other_config.scarf_overhang_threshold
        && config.scarf_joint_speed       == other_config.scarf_joint_speed
        && config.scarf_joint_flow_ratio       == other_config.scarf_joint_flow_ratio
        && config.seam_slope_start_height == other_config.seam_slope_start_height
        && config.seam_slope_entire_loop  == other_config.seam_slope_entire_loop
        && config.seam_slope_min_length   == other_config.seam_slope_min_length
        && config.seam_slope_steps        == other_config.seam_slope_steps
        && config.seam_slope_inner_walls  == other_config.seam_slope_inner_walls;
}

// Orca: The parts of `slices` over the void wide enough to be bridged. Narrower ones, as the overhang a sloped wall
// leaves on each layer, are printed by the walls.
static ExPolygons bridged_overhangs(const Layer &layer, const ExPolygons &slices, coord_t wall_width)
{
    return layer.lower_layer == nullptr ? ExPolygons() : opening_ex(diff_ex(slices, layer.lower_layer->lslices), float(wall_width));
}

// Orca: Each region of a multi-material object, painted or a part of its own, is printed within its own outline, so a
// bridge segmented into a region of its own ends at its neighbours, where a single-material bridge reaches on over the
// supported area around it. Move a strip of the supported neighbours, as wide as their outer wall, to the bridge to
// anchor on, which the infill and the bridge detection then see. The walls of the neighbours are still generated over
// the strip, which is then left to the bridge, see Layer::make_perimeters().
void Layer::anchor_bridges_in_neighbours()
{
    // On the last layer every anchor would be a top surface.
    if (m_regions.size() < 2 || lower_layer == nullptr || upper_layer == nullptr)
        return;
    for (LayerRegion *bridge : m_regions) {
        const float      width     = float(bridge->flow(frExternalPerimeter).scaled_width());
        const ExPolygons slices    = to_expolygons(bridge->slices.surfaces);
        const ExPolygons overhangs = bridged_overhangs(*this, slices, coord_t(width));
        if (overhangs.empty())
            continue;
        // A top surface of a neighbour would show the bridge's filament. The slivers narrower than a wall the outline of
        // a sloped object leaves uncovered are not surfaces to see.
        const ExPolygons top = opening_ex(diff_ex(lslices, upper_layer->lslices), width / 2);
        // The strip replaces the outer wall of the neighbour, which it keeps along the outline of the object.
        ExPolygons anchors;
        for (const LayerRegion *neighbour : m_regions)
            if (neighbour != bridge && ! neighbour->slices.empty()) {
                const float neighbour_width = float(neighbour->flow(frExternalPerimeter).scaled_width());
                Polygons    reach           = intersection(offset(overhangs, neighbour_width), offset(lslices, -neighbour_width));
                if (! top.empty())
                    reach = diff(reach, top);
                append(anchors, intersection_ex(intersection_ex(neighbour->slices.surfaces, lower_layer->lslices), reach));
            }
        if (anchors.empty())
            continue;
        const Polygons inside = offset(lslices, -width);
        // The gaps narrower than a wall the anchors leave in the bridge, as of a thin line of a neighbour between two
        // bridges or of a crack between two regions, go to the bridge too, or they would be walled.
        const ExPolygons anchored = union_ex(slices, anchors);
        ExPolygons gaps = intersection_ex(diff_ex(closing_ex(anchored, width / 2), anchored), intersection(offset(anchors, width), inside));
        // A gap is kept or left whole: cutting it would leave a crack along the bridge, which would be walled.
        if (! top.empty())
            gaps.erase(std::remove_if(gaps.begin(), gaps.end(), [&top](const ExPolygon &gap) { return ! intersection_ex(gap, top).empty(); }),
                       gaps.end());
        append(anchors, std::move(gaps));
        for (LayerRegion *neighbour : m_regions)
            if (neighbour != bridge && ! neighbour->slices.empty())
                neighbour->slices.set(diff_ex(neighbour->slices.surfaces, anchors), stInternal);
        bridge->slices.set(union_ex(slices, anchors), stInternal);
    }
}

// Orca: The parts of the other regions of `layer` within reach of the walls of `layerms` along their overhangs. A bridge
// segmented into a region of its own would be walled in mid air along its neighbours, where a single-material bridge
// has no walls. Generating its walls over these parts too and removing what lands on them leaves the walls to the
// outline of the object, as on a single-material bridge.
static ExPolygons neighbours_along_overhangs(const Layer &layer, const LayerRegionPtrs &layerms)
{
    const LayerRegion &layerm     = *layerms.front();
    const int          wall_loops = layerm.region().config().wall_loops.value;
    if (layer.lower_layer == nullptr || wall_loops <= 0)
        return {};
    ExPolygons own;
    ExPolygons others;
    for (const LayerRegion *region : layer.regions())
        append(std::find(layerms.begin(), layerms.end(), region) == layerms.end() ? others : own, to_expolygons(region->slices.surfaces));
    const coord_t    wall_width = layerm.flow(frExternalPerimeter).scaled_width();
    const ExPolygons overhangs  = others.empty() ? ExPolygons() : bridged_overhangs(layer, own, wall_width);
    if (overhangs.empty())
        return {};
    // Far enough for the walls along the far side of these parts to stay off the anchor of the bridge, and for the
    // extra perimeters on overhangs to find the bridge anchored there, as on a single-material object.
    const float walls = float(wall_width + wall_loops * layerm.flow(frPerimeter).scaled_spacing());
    ExPolygons  out   = intersection_ex(others, offset(overhangs, walls + float(wall_width + scale_(EXTERNAL_INFILL_MARGIN))));
    // Only the neighbours the overhangs hang from, past the strip they may have handed over to anchor on.
    const Polygons along = offset(overhangs, float(2 * wall_width));
    out.erase(std::remove_if(out.begin(), out.end(), [&along](const ExPolygon &expoly) { return intersection(along, expoly).empty(); }), out.end());
    // Along the outline of the object, the neighbours may be thinner than the walls. Add the void past their outline,
    // so that the walls along it are removed too. The void past the outline of these regions keeps their walls.
    if (! out.empty())
        append(out, diff_ex(offset(out, walls), union_(to_polygons(layer.lslices), offset(own, float(wall_width / 2)))));
    return out;
}

// Orca: The runs of `polyline` outside `area`, in the order and the direction it is extruded in. Clipping a polyline
// crossing the boundary of `area` does not keep those, so it is clipped segment by segment.
static Polylines runs_outside(const Polyline &polyline, const Polygons &area)
{
    if (const Polylines whole = diff_pl(polyline, area); whole.empty() || std::abs(total_length(whole) - polyline.length()) < SCALED_EPSILON)
        return whole.empty() ? Polylines() : Polylines{ polyline };
    Polylines runs;
    for (size_t i = 1; i < polyline.size(); ++ i) {
        const Point &a      = polyline.points[i - 1];
        const Point &b      = polyline.points[i];
        const Vec2d  dir    = (b - a).cast<double>();
        Polylines    pieces = diff_pl(Polyline{ a, b }, area);
        for (Polyline &piece : pieces)
            if (dir.dot((piece.last_point() - piece.first_point()).cast<double>()) < 0.)
                piece.reverse();
        std::sort(pieces.begin(), pieces.end(), [&a, &dir](const Polyline &l, const Polyline &r) {
            return dir.dot((l.first_point() - a).cast<double>()) < dir.dot((r.first_point() - a).cast<double>());
        });
        for (Polyline &piece : pieces) {
            if (! runs.empty() && runs.back().last_point() == piece.first_point())
                runs.back().append(piece.last_point());
            else
                runs.emplace_back(std::move(piece));
        }
    }
    return runs;
}

// Orca: Removes the parts of the extrusions over `area`. One crossing its boundary is split into the runs left outside.
static void remove_extrusions_over(ExtrusionEntityCollection &collection, const Polygons &area)
{
    ExtrusionEntitiesPtr kept;
    for (ExtrusionEntity *entity : collection.entities) {
        if (auto *child = dynamic_cast<ExtrusionEntityCollection*>(entity)) {
            remove_extrusions_over(*child, area);
            if (child->empty())
                delete child;
            else
                kept.emplace_back(child);
            continue;
        }
        ExtrusionPaths paths;
        if (auto *loop = dynamic_cast<ExtrusionLoop*>(entity))
            paths = loop->paths;
        else if (auto *multi_path = dynamic_cast<ExtrusionMultiPath*>(entity))
            paths = multi_path->paths;
        else if (auto *path = dynamic_cast<ExtrusionPath*>(entity))
            paths.emplace_back(*path);
        std::vector<ExtrusionPaths> runs;
        double                      length = 0.;
        for (const ExtrusionPath &path : paths)
            for (Polyline &run : runs_outside(path.polyline.to_polyline(), area)) {
                length += run.length();
                if (runs.empty() || runs.back().back().last_point() != run.first_point())
                    runs.emplace_back();
                runs.back().emplace_back(Polyline3(run), path);
            }
        if (paths.empty() || std::abs(length - entity->length()) < SCALED_EPSILON) {
            kept.emplace_back(entity);
            continue;
        }
        // A loop cut away from its start runs on from its last run into its first one.
        if (entity->is_loop() && runs.size() > 1 && runs.back().back().last_point() == runs.front().front().first_point()) {
            append(runs.back(), std::move(runs.front()));
            runs.front() = std::move(runs.back());
            runs.pop_back();
        }
        for (ExtrusionPaths &run : runs) {
            const double run_length = std::accumulate(run.begin(), run.end(), 0., [](double l, const ExtrusionPath &p) { return l + p.length(); });
            // A stub no longer than it is wide would only leave a blob.
            if (run_length <= scale_(run.front().width))
                continue;
            ExtrusionEntity *clipped;
            if (run.size() == 1) {
                clipped = new ExtrusionPath(std::move(run.front()));
            } else {
                auto *multi_path  = new ExtrusionMultiPath();
                multi_path->paths = std::move(run);
                clipped           = multi_path;
            }
            clipped->inset_idx = entity->inset_idx;
            kept.emplace_back(clipped);
        }
        delete entity;
    }
    collection.entities = std::move(kept);
}

// Here the perimeters are created cummulatively for all layer regions sharing the same parameters influencing the perimeters.
// The perimeter paths and the thin fills (ExtrusionEntityCollection) are assigned to the first compatible layer region.
// The resulting fill surface is split back among the originating regions.
void Layer::make_perimeters()
{
    BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id();

    const auto clear_generated_extrusions = [](LayerRegion *layer_region) {
        layer_region->perimeters.clear();
        layer_region->fills.clear();
        layer_region->thin_fills.clear();
    };

    // keep track of regions whose perimeters we have already generated
    std::vector<unsigned char> done(m_regions.size(), false);

    for (LayerRegionPtrs::iterator layerm = m_regions.begin(); layerm != m_regions.end(); ++ layerm)
    	if ((*layerm)->slices.empty()) {
 			(*layerm)->perimeters.clear();
 			(*layerm)->fills.clear();
 			(*layerm)->thin_fills.clear();
    	} else {
	        size_t region_id = layerm - m_regions.begin();
	        if (done[region_id])
	            continue;
	        BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id() << ", region " << region_id;
	        done[region_id] = true;
	        const PrintRegion &this_region = (*layerm)->region();

	        // find compatible regions
	        LayerRegionPtrs layerms;
	        layerms.push_back(*layerm);
	        for (LayerRegionPtrs::const_iterator it = layerm + 1; it != m_regions.end(); ++it)
	            if (! (*it)->slices.empty()) {
		            LayerRegion* other_layerm = *it;
		            const PrintRegion &other_region = other_layerm->region();
                    // Per-part gradient tags a region with its owning ModelVolume; merging two
                    // differently-tagged regions would collapse volumes that need independent
                    // gradient runs. Both tags are invalid unless per-part gradient is on, so
                    // this is a no-op for every other configuration.
                    if (this_region.gradient_volume_id() != other_region.gradient_volume_id())
                        continue;
                    if (is_perimeter_compatible(*m_object->print(), this_region, other_region))
                    {
                        clear_generated_extrusions(other_layerm);
                        layerms.push_back(other_layerm);
                        done[it - m_regions.begin()] = true;
                    }
		        }

	        // Orca: The walls of these regions are generated over the neighbours along their bridges and over the strips they
	        // handed to bridges of other regions to anchor on, then removed from there.
	        ExPolygons clipped = neighbours_along_overhangs(*this, layerms);
	        Polygons   group_slices;
	        for (const LayerRegion *l : layerms)
	            polygons_append(group_slices, to_polygons(l->slices.surfaces));
	        for (const LayerRegion *l : layerms)
	            if (const ExPolygons slices = to_expolygons(l->slices.surfaces); slices != l->raw_slices)
	                // A strip handed to a region of this group keeps the walls they share.
	                append(clipped, diff_ex(l->raw_slices, group_slices));
	        clipped = union_ex(clipped);
	        if (layerms.size() == 1 && clipped.empty()) {  // optimization
	            (*layerm)->fill_surfaces.surfaces.clear();
                (*layerm)->make_perimeters((*layerm)->slices, {*layerm}, &(*layerm)->fill_surfaces, &(*layerm)->fill_no_overlap_expolygons);
	            (*layerm)->fill_expolygons = to_expolygons((*layerm)->fill_surfaces.surfaces);
	        } else {
	            // Orca: Unlike the compatible regions above, the initiating region has not
	            // been cleared yet and may contain paths from a previous incompatible run.
	            clear_generated_extrusions(*layerm);

	            SurfaceCollection new_slices;
	            // Use the region with highest infill rate, as the make_perimeters() function below decides on the gap fill based on the infill existence.
	            LayerRegion *layerm_config = layerms.front();
	            {
	                // group slices (surfaces) according to number of extra perimeters
	                std::map<unsigned short, Surfaces> slices;  // extra_perimeters => [ surface, surface... ]
	                for (LayerRegion *layerm : layerms) {
	                    for (const Surface &surface : layerm->slices.surfaces)
	                        slices[surface.extra_perimeters].emplace_back(surface);
	                    if (layerm->region().config().sparse_infill_density > layerm_config->region().config().sparse_infill_density)
	                    	layerm_config = layerm;
	                }
	                for (const ExPolygon &expoly : clipped)
	                    slices.begin()->second.emplace_back(slices.begin()->second.front(), expoly);
	                // merge the surfaces assigned to each group
	                for (std::pair<const unsigned short,Surfaces> &surfaces_with_extra_perimeters : slices)
	                    new_slices.append(offset_ex(surfaces_with_extra_perimeters.second, ClipperSafetyOffset), surfaces_with_extra_perimeters.second.front());
	            }

	            // make perimeters
	            SurfaceCollection fill_surfaces;
                //BBS
                ExPolygons fill_no_overlap;
	            layerm_config->make_perimeters(new_slices, layerms, &fill_surfaces, &fill_no_overlap);
	            if (! clipped.empty()) {
	                // The fill there is dropped below, with the fill outside the slices of these regions.
	                remove_extrusions_over(layerm_config->perimeters, to_polygons(clipped));
	                remove_extrusions_over(layerm_config->thin_fills, to_polygons(clipped));
	            }

	            // assign fill_surfaces to each layer, clearing those of a previous run first, which an empty fill would keep
	            for (LayerRegion *l : layerms) {
	                l->fill_expolygons.clear();
	                l->fill_surfaces.clear();
	                l->fill_no_overlap_expolygons.clear();
	            }
	            if (!fill_surfaces.surfaces.empty()) {
	                for (LayerRegionPtrs::iterator l = layerms.begin(); l != layerms.end(); ++l) {
	                    // Separate the fill surfaces.
	                    ExPolygons expp = intersection_ex(fill_surfaces.surfaces, (*l)->slices.surfaces);
	                    (*l)->fill_expolygons = expp;
	                    (*l)->fill_surfaces.set(std::move(expp), fill_surfaces.surfaces.front());
                        //BBS: Separate fill_no_overlap
                        (*l)->fill_no_overlap_expolygons = intersection_ex((*l)->slices.surfaces, fill_no_overlap);
	                }

	                // When counterbore hole bridging (chbFilled) is active, process_no_bridge may
	                // create fill surfaces that extend beyond all region slices (e.g. by clearing
	                // holes in the bridge expolygon). These "extra" fills are lost during the
	                // intersection-based splitting above. Recover them and assign to the first
	                // merged region so the sacrificial bridge layer is not broken.
	                if (layerm_config->region().config().counterbore_hole_bridging.value != chbNone) {
	                    Polygons all_region_slices_p;
	                    for (LayerRegion *l : layerms)
	                        polygons_append(all_region_slices_p, to_polygons(l->slices.surfaces));
	                    polygons_append(all_region_slices_p, to_polygons(clipped));
	                    ExPolygons extra_fill = diff_ex(fill_surfaces.surfaces, all_region_slices_p, ApplySafetyOffset::Yes);
	                    if (!extra_fill.empty()) {
	                        append(layerms.front()->fill_expolygons, extra_fill);
	                        layerms.front()->fill_expolygons = union_ex(layerms.front()->fill_expolygons);
	                        layerms.front()->fill_surfaces.append(std::move(extra_fill), fill_surfaces.surfaces.front());
	                    }
	                }
	            }
	        }
	    }
    BOOST_LOG_TRIVIAL(trace) << "Generating perimeters for layer " << this->id() << " - Done";
}

// Orca: Appends the runs of the closed `loop` inside `zone` to `out`, as a loop if all of it is inside.
static void append_refinement_loop(const Polygon &loop, const ExPolygons &zone, const ExtrusionPath &templ, ExtrusionEntityCollection &out)
{
    // Start the loop outside the zone if it leaves it, so that no run is cut at the first point.
    const auto outside = std::find_if(loop.points.begin(), loop.points.end(), [&zone](const Point &point) {
        return std::none_of(zone.begin(), zone.end(), [&point](const ExPolygon &expoly) { return expoly.contains(point); });
    });
    const Polyline source = outside == loop.points.end() ? loop.split_at_first_point() : loop.split_at_index(int(outside - loop.points.begin()));
    Polylines      runs   = intersection_pl(source, to_polygons(zone));
    if (runs.empty())
        return;
    const bool whole = outside == loop.points.end() && runs.size() == 1;
    if (! whole && outside != loop.points.end())
        restore_source_path_order(source, runs);
    for (const Polyline &run : runs) {
        // A stub no longer than it is wide would only leave a blob.
        if (run.length() <= scale_(templ.width))
            continue;
        ExtrusionPath path(Polyline3(run), templ);
        if (whole)
            out.append(ExtrusionLoop(std::move(path), loop.is_clockwise() ? elrHole : elrDefault));
        else
            out.append(std::move(path));
        out.entities.back()->inset_idx = templ.inset_idx;
    }
}

// Orca: Distance of a point to a fine zone, 0 inside it. The joints of a fine wall ramp over this distance.
class ZoneDistance
{
public:
    explicit ZoneDistance(const Polygons &zone) : m_distancer(to_lines(zone)) {}
    // In scaled units.
    double operator()(const Point &point) const { return std::max(0., double(m_distancer.distance_from_lines<true>(point))); }
    double operator()(const Point3 &point) const { return (*this)(Point(point.x(), point.y())); }

private:
    AABBTreeLines::LinesDistancer<Line> m_distancer;
};

// Orca: `points` with the segments that come within `reach` of the zone split into pieces no longer than `step`, all
// scaled, so that a ramp over them follows its distance to the zone.
static Points3 subdivide_near_zone(const Points3 &points, const ZoneDistance &distance, double reach, double step)
{
    Points3 out;
    if (points.empty())
        return out;
    out.reserve(points.size());
    out.emplace_back(points.front());
    for (size_t idx = 1; idx < points.size(); ++ idx) {
        const Point3 &a      = points[idx - 1];
        const Point3 &b      = points[idx];
        const Vec3d   ab     = (b - a).cast<double>();
        const double  length = ab.head<2>().norm();
        if (length > step && std::min(distance(a), distance(b)) < reach + length) {
            const int num = int(std::ceil(length / step));
            for (int piece = 1; piece < num; ++ piece) {
                const Vec3d p = a.cast<double>() + ab * (double(piece) / num);
                out.emplace_back(coord_t(std::round(p.x())), coord_t(std::round(p.y())), coord_t(std::round(p.z())));
            }
        }
        out.emplace_back(b);
    }
    return out;
}

// Orca: The wall of the layer over a joint ramps from the height of a pass at the fine zone up to the layer height
// `ramp` further on, as the top line of the merge pass of the prototype (scripts/layer_combination_coupons.py). Leaving
// the fine zone, the ramp is pre-pressurised: it rises as u^0.8 with a bump of up to 10 %, entering it, it is linear.
static ExtrusionPaths ramp_outer_wall(const ExtrusionPath &path, const ZoneDistance &distance, double ramp, double pass_height)
{
    const double layer_height = path.height;
    if (ramp <= 0. || layer_height <= pass_height + EPSILON || path.polyline.points.size() < 2)
        return { path };
    const double  scaled_ramp = scale_(ramp);
    const Points3 points      = subdivide_near_zone(path.polyline.points, distance, scaled_ramp, scale_(0.2));
    const bool    leaving     = distance(points.front()) < distance(points.back());
    ExtrusionPaths out;
    for (size_t idx = 1; idx < points.size(); ++ idx) {
        const Point  middle((points[idx - 1].x() + points[idx].x()) / 2, (points[idx - 1].y() + points[idx].y()) / 2);
        const double u      = std::min(1., distance(middle) / scaled_ramp);
        const double rise   = leaving ? std::pow(u, 0.8) * (1. + 0.1 * std::sin(PI * u)) : u;
        const double height = u >= 1. ? layer_height : pass_height + (layer_height - pass_height) * rise;
        if (! out.empty() && std::abs(out.back().height - height) < 1e-3) {
            out.back().polyline.points.emplace_back(points[idx]);
            continue;
        }
        out.emplace_back(Polyline3(Points3{ points[idx - 1], points[idx] }), path);
        out.back().height     = float(height);
        out.back().mm3_per_mm = path.mm3_per_mm * height / layer_height;
    }
    if (out.size() == 1 && std::abs(out.front().height - layer_height) < 1e-3)
        return { path };
    return out;
}

// Orca: Ramps the outer walls in `collection` over the joints with a fine zone.
static void ramp_outer_walls(ExtrusionEntityCollection &collection, const ZoneDistance &distance, double ramp, double pass_height)
{
    for (ExtrusionEntity *&entity : collection.entities) {
        if (auto *child = dynamic_cast<ExtrusionEntityCollection*>(entity)) {
            ramp_outer_walls(*child, distance, ramp, pass_height);
            continue;
        }
        ExtrusionPaths *paths = nullptr;
        if (auto *loop = dynamic_cast<ExtrusionLoop*>(entity))
            paths = &loop->paths;
        else if (auto *multi_path = dynamic_cast<ExtrusionMultiPath*>(entity))
            paths = &multi_path->paths;
        else if (auto *path = dynamic_cast<ExtrusionPath*>(entity); path != nullptr && path->role() == erExternalPerimeter) {
            // A single path split in parts becomes a multi path.
            ExtrusionPaths split = ramp_outer_wall(*path, distance, ramp, pass_height);
            if (split.size() > 1) {
                auto *multi      = new ExtrusionMultiPath();
                multi->paths     = std::move(split);
                multi->inset_idx = entity->inset_idx;
                delete entity;
                entity = multi;
            }
            continue;
        }
        if (paths == nullptr)
            continue;
        ExtrusionPaths out;
        for (const ExtrusionPath &path : *paths)
            append(out, path.role() == erExternalPerimeter ? ramp_outer_wall(path, distance, ramp, pass_height) : ExtrusionPaths{ path });
        *paths = std::move(out);
    }
}

// Orca: Lowers the run-on of the pass `k` (1 for the lowest) of `count` past the fine zone into the wedge of the
// prototype's scarf: the passes under the ramping wall of the layer thin out towards the end of the ramp, and the wedge
// they form together leaves the wall of the layer above it as much room as the ramp takes.
static void wedge_pass(ExtrusionEntityCollection &extrusions, const ZoneDistance &distance, double ramp, double pass_height,
                       double min_height, int count, int k)
{
    for (ExtrusionEntity *entity : extrusions.entities) {
        auto *path = dynamic_cast<ExtrusionPath*>(entity);
        if (path == nullptr || path->polyline.points.size() < 2)
            continue;
        Points3 points  = subdivide_near_zone(path->polyline.points, distance, scale_(ramp), scale_(0.2));
        bool    lowered = false;
        for (Point3 &point : points) {
            const double u     = unscale<double>(distance(point));
            const double wedge = (count - 1) * pass_height * std::max(0., 1. - u / ramp);
            const double top   = std::min(k * pass_height, std::max((k - 1) * pass_height + min_height, wedge));
            point.z()          = coord_t(scale_(top - k * pass_height));
            lowered            = lowered || point.z() != 0;
        }
        if (lowered) {
            path->polyline    = Polyline3(points);
            path->z_contoured = true;
        }
    }
}

// Orca: The paths of an open extrusion.
static ExtrusionPaths paths_of(const ExtrusionEntity &entity)
{
    if (const auto *multi_path = dynamic_cast<const ExtrusionMultiPath*>(&entity))
        return multi_path->paths;
    if (const auto *path = dynamic_cast<const ExtrusionPath*>(&entity))
        return { *path };
    return {};
}

static void reverse_paths(ExtrusionPaths &paths)
{
    std::reverse(paths.begin(), paths.end());
    for (ExtrusionPath &path : paths)
        path.reverse();
}

// Orca: Appends `next` to `chain`, bridging the small gap between the end of one and the start of the other.
static void append_to_chain(ExtrusionPaths &chain, ExtrusionPaths &&next)
{
    if (next.empty())
        return;
    if (! chain.empty() && chain.back().last_point() != next.front().first_point()) {
        const Point end = chain.back().last_point();
        next.front().polyline.points.insert(next.front().polyline.points.begin(), Point3(end.x(), end.y(), coord_t(0)));
    }
    append(chain, std::move(next));
}

// Orca: The open outer walls in `collection`, by their collection and index.
static void collect_open_outer_walls(ExtrusionEntityCollection &collection, std::vector<std::pair<ExtrusionEntityCollection*, size_t>> &out)
{
    for (size_t idx = 0; idx < collection.entities.size(); ++ idx) {
        ExtrusionEntity *entity = collection.entities[idx];
        if (auto *child = dynamic_cast<ExtrusionEntityCollection*>(entity))
            collect_open_outer_walls(*child, out);
        else if (! entity->is_loop() && entity->role() == erExternalPerimeter)
            out.emplace_back(&collection, idx);
    }
}

// Orca: Splices the open stretches of the top pass of a fine wall with the outer walls of the layer they meet into one
// line, as the merge pass of the prototype: the top of every layer is one line through fine and coarse, without a start
// or an end at the joints. The stretches spliced are taken out of `stretches`.
static void splice_top_passes(std::vector<LayerRegion*> &regions, std::vector<std::unique_ptr<ExtrusionPath>> &stretches)
{
    std::vector<std::pair<ExtrusionEntityCollection*, size_t>> walls;
    for (LayerRegion *layerm : regions)
        collect_open_outer_walls(layerm->perimeters, walls);
    if (walls.empty() || stretches.empty())
        return;

    std::vector<bool> wall_used(walls.size(), false);
    const auto near = [](const Point &a, const Point &b, double tolerance) { return (a - b).cast<double>().squaredNorm() <= tolerance * tolerance; };
    // The nearest unused stretch with an end within `tolerance` of `point`, and whether it ends there.
    const auto find_stretch = [&stretches](const Point &point, double tolerance) -> std::pair<int, bool> {
        int    best     = -1;
        bool   reversed = false;
        double best_sqr = tolerance * tolerance;
        for (size_t idx = 0; idx < stretches.size(); ++ idx)
            if (stretches[idx]) {
                const double first = (stretches[idx]->first_point() - point).cast<double>().squaredNorm();
                const double last  = (stretches[idx]->last_point() - point).cast<double>().squaredNorm();
                if (first <= best_sqr) { best = int(idx); reversed = false; best_sqr = first; }
                if (last <= best_sqr) { best = int(idx); reversed = true; best_sqr = last; }
            }
        return { best, reversed };
    };
    const auto find_wall = [&walls, &wall_used](const Point &point, double tolerance) -> std::pair<int, bool> {
        int    best     = -1;
        bool   reversed = false;
        double best_sqr = tolerance * tolerance;
        for (size_t idx = 0; idx < walls.size(); ++ idx)
            if (! wall_used[idx]) {
                const ExtrusionEntity &wall  = *walls[idx].first->entities[walls[idx].second];
                const double           first = (wall.first_point() - point).cast<double>().squaredNorm();
                const double           last  = (wall.last_point() - point).cast<double>().squaredNorm();
                if (first <= best_sqr) { best = int(idx); reversed = false; best_sqr = first; }
                if (last <= best_sqr) { best = int(idx); reversed = true; best_sqr = last; }
            }
        return { best, reversed };
    };

    std::vector<ExtrusionEntity**> removed;
    for (size_t start = 0; start < walls.size(); ++ start) {
        if (wall_used[start])
            continue;
        wall_used[start]             = true;
        ExtrusionEntity *&start_slot = walls[start].first->entities[walls[start].second];
        ExtrusionPaths    chain      = paths_of(*start_slot);
        if (chain.empty())
            continue;
        const double         tolerance = 2. * scale_(chain.front().width);
        std::vector<size_t>  chain_walls;
        bool                 spliced = false;
        bool                 closed  = false;
        // Extends the chain at its end: a stretch, then a wall, and so on, until it closes or nothing meets it.
        const auto extend = [&]() {
            for (;;) {
                const auto [stretch, stretch_reversed] = find_stretch(chain.back().last_point(), tolerance);
                if (stretch < 0)
                    return;
                ExtrusionPaths next { *stretches[stretch] };
                if (stretch_reversed)
                    reverse_paths(next);
                append_to_chain(chain, std::move(next));
                stretches[stretch].reset();
                spliced = true;
                if (near(chain.back().last_point(), chain.front().first_point(), tolerance)) {
                    closed = true;
                    return;
                }
                const auto [wall, wall_reversed] = find_wall(chain.back().last_point(), tolerance);
                if (wall < 0)
                    return;
                ExtrusionPaths next_wall = paths_of(*walls[wall].first->entities[walls[wall].second]);
                if (wall_reversed)
                    reverse_paths(next_wall);
                append_to_chain(chain, std::move(next_wall));
                wall_used[wall] = true;
                chain_walls.emplace_back(size_t(wall));
            }
        };
        extend();
        if (! closed) {
            reverse_paths(chain);
            extend();
        }
        if (! spliced)
            continue;

        ExtrusionEntity *line = nullptr;
        if (closed) {
            const Point first = chain.front().first_point();
            if (chain.back().last_point() != first)
                chain.back().polyline.points.emplace_back(first.x(), first.y(), coord_t(0));
            Polygon polygon;
            for (const ExtrusionPath &path : chain)
                for (const Point3 &point : path.polyline.points)
                    polygon.points.emplace_back(point.x(), point.y());
            line = new ExtrusionLoop(std::move(chain), polygon.is_clockwise() ? elrHole : elrDefault);
        } else {
            auto *multi_path  = new ExtrusionMultiPath();
            multi_path->paths = std::move(chain);
            line              = multi_path;
        }
        line->inset_idx = start_slot->inset_idx;
        delete start_slot;
        start_slot = line;
        for (size_t wall : chain_walls)
            removed.emplace_back(&walls[wall].first->entities[walls[wall].second]);
    }
    for (ExtrusionEntity **slot : removed) {
        delete *slot;
        *slot = nullptr;
    }
    for (LayerRegion *layerm : regions) {
        const auto erase_null = [](ExtrusionEntityCollection &collection, const auto &self) -> void {
            collection.entities.erase(std::remove(collection.entities.begin(), collection.entities.end(), nullptr), collection.entities.end());
            for (ExtrusionEntity *entity : collection.entities)
                if (auto *child = dynamic_cast<ExtrusionEntityCollection*>(entity))
                    self(*child, self);
        };
        erase_null(layerm->perimeters, erase_null);
    }
}

// Orca: Index of the island of `islands` that holds `point`, or else the closest one, -1 if there are none.
static int island_of(const ExPolygons &islands, const Point &point)
{
    int    closest     = -1;
    double closest_sqr = std::numeric_limits<double>::max();
    for (size_t idx = 0; idx < islands.size(); ++ idx) {
        if (islands[idx].contains(point))
            return int(idx);
        if (const Point *nearest = islands[idx].contour.closest_point(point); nearest != nullptr)
            if (const double sqr = (*nearest - point).cast<double>().squaredNorm(); sqr < closest_sqr) {
                closest     = int(idx);
                closest_sqr = sqr;
            }
    }
    return closest;
}

using RefinementIslands = std::vector<std::vector<std::pair<int, std::unique_ptr<ExtrusionEntityCollection>>>>;

// Orca: Takes the islands whose passes are each one closed outer wall out of `pass_islands` (lines of each pass by
// island, lowest pass first) and appends them to `out` as helices rising from the lowest pass to print_z: a flat turn
// on the layer below, a turn that ramps the flow up as it rises off it, a turn for each further pass, and a flat
// turn at print_z that ramps the flow down, so that the bottom and the top of the layer stay flat. Each turn follows
// the outline of its pass, opened at the point closest to the back of the top one.
static void make_refinement_helices(RefinementIslands &pass_islands, double pass_height, std::vector<LayerRegion::RefinementPass> &out)
{
    const int  count       = int(pass_islands.size());
    const auto single_loop = [](const ExtrusionEntityCollection &extrusions) -> const ExtrusionLoop* {
        if (extrusions.entities.size() != 1)
            return nullptr;
        const auto *loop = dynamic_cast<const ExtrusionLoop*>(extrusions.entities.front());
        return loop != nullptr && loop->paths.size() == 1 && loop->role() == erExternalPerimeter ? loop : nullptr;
    };
    for (const auto &[island, top] : pass_islands.back()) {
        if (island < 0)
            continue;
        // The lines of this island in each pass, if they are one loop in each.
        std::vector<std::unique_ptr<ExtrusionEntityCollection>*> stack;
        for (auto &pass : pass_islands) {
            std::unique_ptr<ExtrusionEntityCollection> *found = nullptr;
            size_t                                      num   = 0;
            for (auto &[other, extrusions] : pass)
                if (other == island && extrusions) {
                    found = &extrusions;
                    ++ num;
                }
            if (num != 1 || single_loop(**found) == nullptr)
                break;
            stack.emplace_back(found);
        }
        if (int(stack.size()) != count)
            continue;

        const Polygon top_polygon = single_loop(**stack.back())->polygon();
        const Point   seam        = *std::max_element(top_polygon.points.begin(), top_polygon.points.end(),
            [](const Point &l, const Point &r) { return l.y() < r.y() || (l.y() == r.y() && l.x() > r.x()); });
        LayerRegion::RefinementPass helix { - (count - 1) * pass_height };
        for (int turn = 0; turn <= count; ++ turn) {
            const int            pass    = std::min(turn, count - 1);
            const ExtrusionLoop &loop    = *single_loop(**stack[pass]);
            const Polygon        polygon = loop.polygon();
            Polyline3            line(polygon.split_at_index(polygon.closest_point_index(seam)));
            const ExtrusionPath &templ   = loop.paths.front();
            ExtrusionPath       *path    = nullptr;
            if (turn == 0)
                path = new ExtrusionPath(std::move(line), templ);
            else if (turn < count)
                path = new ExtrusionPathSloped(std::move(line), templ, { 0., turn == 1 ? 0. : 1. }, { 1., 1. });
            else
                path = new ExtrusionPathSloped(std::move(line), templ, { 1., 1. }, { 1., 0. });
            path->inset_idx = 0;
            helix.extrusions.entities.emplace_back(path);
            helix.turn_offsets.emplace_back(- (count - 1 - pass) * pass_height);
        }
        helix.islands.assign(helix.extrusions.entities.size(), island);
        for (std::unique_ptr<ExtrusionEntityCollection> *extrusions : stack)
            extrusions->reset();
        out.emplace_back(std::move(helix));
    }
}

// The outer surface of a refined region is printed in `count` passes, each a 1/count of the layer high and following
// the outline of the object at its own height. Behind them the layer is printed as it is, at its full height.
//
// The passes fill the fine zone: what any pass covers, less the core, which is where the layer keeps its own
// extrusions. The core lies behind the outer wall of the layer, so the passes replace the outer wall only and the inner
// walls keep the layer height. Where a pass outline lies further out, the zone widens to it; where it lies further in,
// only that stretch of the inner walls is cut out of the core. A feature too thin for a core is printed by the passes
// alone.
void Layer::apply_refinement(const std::map<int, std::vector<ExPolygons>> &sub_slices)
{
    // Regions refined into the same number of passes are refined as one, with the settings of the first of them.
    struct Group
    {
        LayerRegion *layerm { nullptr };
        ExPolygons   area;
    };
    std::map<int, Group> groups;
    ExPolygons           unrefined;
    for (LayerRegion *layerm : m_regions) {
        if (layerm->slices.empty())
            continue;
        const PrintRegionConfig &config = layerm->region().config();
        // A fuzzy wall keeps the layer height, and so does a region without walls.
        const bool fuzzy   = config.fuzzy_skin.value != FuzzySkinType::None && config.fuzzy_skin.value != FuzzySkinType::Disabled_fuzzy;
        const int  count   = fuzzy || config.wall_loops.value < 1 ? 1 : layer_refinement_pass_count(height, config.layer_refinement_height.value);
        if (count < 2 || sub_slices.find(count) == sub_slices.end()) {
            append(unrefined, to_expolygons(layerm->slices.surfaces));
            continue;
        }
        Group &group = groups[count];
        if (group.layerm == nullptr)
            group.layerm = layerm;
        append(group.area, to_expolygons(layerm->slices.surfaces));
    }
    if (groups.empty())
        return;

    const Polygons outline = to_polygons(lslices);
    // Where the passes replace the extrusions of the layer.
    Polygons fine_zones;
    // The ramps of the joints: the fine zone they start from, their length and the height of a pass.
    struct Ramp
    {
        Polygons zone;
        double   length;
        double   pass_height;
    };
    std::vector<Ramp> ramps;
    // The open stretches of the top passes, spliced with the walls of the layer they meet into one line.
    std::vector<std::pair<LayerRegion*, std::pair<int, std::unique_ptr<ExtrusionPath>>>> top_stretches;

    for (auto &[count, group] : groups) {
        LayerRegion                   &layerm      = *group.layerm;
        // The outline under the lowest pass, then those of the passes.
        const std::vector<ExPolygons> &outlines    = sub_slices.find(count)->second;
        const ExPolygons              &below       = outlines.front();
        const std::vector<ExPolygons>  pass_slices(outlines.begin() + 1, outlines.end());
        const PrintRegionConfig       &config      = layerm.region().config();
        const double                   pass_height = height / count;
        const Flow                     ext_flow    = layerm.flow(frExternalPerimeter);
        const Flow                     flow        = layerm.flow(frPerimeter);
        // A wall of a pass takes the place of a wall of the layer: its spacing, and the overlap of a thin line with
        // a thick one.
        const float overlap       = float(0.5 * (1. - 0.25 * PI) * (pass_height + height) * config.layer_refinement_wall_overlap.value / 100.);
        const Flow  fine_ext_flow = layerm.flow(frExternalPerimeter, pass_height).with_width(ext_flow.spacing() + overlap);
        const Flow  fine_flow     = layerm.flow(frPerimeter, pass_height).with_width(flow.spacing() + overlap);
        // Depth of the boundary between the outer wall of the layer and its first inner wall, and from one inner wall
        // to the next.
        const coord_t wall_depth = (ext_flow.scaled_width() + ext_flow.scaled_spacing()) / 2;
        const coord_t wall_step  = flow.scaled_spacing();
        const int     wall_loops = config.wall_loops.value;

        const ExPolygons area = union_ex(group.area);
        // The outlines of the passes on a slope lie past the outline of the layer: grow the area outwards only.
        const float reach     = float(std::max(ext_flow.scaled_width(), coord_t(scale_(3. * height))));
        // It grows only in front of the region, not along the wall past its ends into the regions next to it.
        const Polygons others    = diff(outline, to_polygons(area));
        Polygons       zone_area = union_(to_polygons(area), diff(diff(offset(area, reach), outline), offset(others, reach)));

        // What all the passes cover, and what any of them covers.
        Polygons common = to_polygons(pass_slices.front());
        Polygons all    = common;
        for (size_t pass = 1; pass < pass_slices.size(); ++ pass) {
            const Polygons slices = to_polygons(pass_slices[pass]);
            common = intersection(common, slices);
            polygons_append(all, slices);
        }
        all = union_(all);

        // With a minimum step, only where the outlines of the passes spread apart by it, as on a slope, is refined,
        // through the depth of the walls. Gaps shorter than a millimetre between such stretches are refined too, so
        // that the joints do not pile up on a curved outline.
        bool restricted = false;
        if (const double min_step = config.layer_refinement_min_step.value; min_step > 0.) {
            const Polygons steps = opening(intersection(diff(all, common), zone_area), float(scale_(min_step) / 2.));
            if (steps.empty())
                continue;
            const float depth = float(reach + wall_depth + wall_loops * wall_step);
            zone_area  = intersection(zone_area, closing(offset(steps, depth), float(scale_(1.))));
            restricted = true;
        }

        // A flat top or bottom inside the layer, where the outline of one pass lies far past that of the next, is a
        // surface of the layer for its top or bottom infill, not a step of the wall: the passes leave it, except for the
        // outer wall of the layer along it.
        {
            Polygons flats;
            for (size_t pass = 0; pass + 1 < pass_slices.size(); ++ pass) {
                const Polygons lower = to_polygons(pass_slices[pass]);
                const Polygons upper = to_polygons(pass_slices[pass + 1]);
                polygons_append(flats, opening(diff(lower, upper), float(scale_(1.))));
                polygons_append(flats, opening(diff(upper, lower), float(scale_(1.))));
            }
            if (! flats.empty())
                zone_area = diff(zone_area, diff(union_(flats), diff(union_(all, outline), offset(outline, - float(wall_depth)))));
        }

        // The core starts behind the outer wall of the layer, so that its inner walls stay at the layer height. Only
        // where the outer wall of a pass would run over an inner wall, as where the outline moves inwards, is that
        // stretch cut out of the core, allowing the outlines a quarter of a wall of deviation. A feature too thin for
        // a core is cut out whole.
        const coord_t  tolerance = ext_flow.scaled_width() / 4;
        const Polygons safe      = offset(common, - float(wall_depth - tolerance));
        const Polygons behind    = offset(outline, - float(wall_depth));
        const Polygons core      = diff(behind, opening(diff(intersection(behind, zone_area), safe), float(tolerance / 2)));

        const Polygons shell = diff(all, core);
        const Polygons zone = intersection(zone_area, shell);
        if (zone.empty())
            continue;
        polygons_append(fine_zones, zone);
        polygons_append(refinement_lowered, diff(zone, to_polygons(pass_slices.back())));
        append(refinement_top_support, count > 1 ? pass_slices[count - 2] : below);
        // Where the fine wall ends on a wall carrying on at the layer height, the joint is the scarf of the prototype
        // (scripts/layer_combination_coupons.py): over the ramp length, the wall of the layer rises from the height of a
        // pass to its own, and under it the lower passes run on as a wedge thinning out towards the end of the ramp, the
        // lowest the furthest. The passes also run on by the joint overlap. The run-on is measured from the fine zone,
        // not from the region: only past the ends of the fine wall along the wall.
        const bool   joints     = ! unrefined.empty() || restricted;
        const double ramp       = joints ? config.layer_refinement_scarf_length.value : 0.;
        const double min_height = std::max(0.015, 0.4 * pass_height);
        // A copy, as a structured binding cannot be captured.
        const int    num_passes = count;
        const auto   run_on     = [&config, ramp, min_height, pass_height, num_passes](int pass) {
            double wedge = 0.;
            if (ramp > 0. && pass + 1 < num_passes)
                wedge = std::clamp(ramp * ((num_passes - pass - 1) - min_height / pass_height) / (num_passes - 1), 0., ramp);
            return float(scale_(config.layer_refinement_joint_overlap.value + wedge));
        };
        if (ramp > 0.)
            ramps.push_back({ zone, ramp, pass_height });

        // The passes are extruded with the outer wall filament, all of their lines.
        ExtrusionPath ext_templ(erExternalPerimeter, fine_ext_flow.mm3_per_mm(), fine_ext_flow.width(), fine_ext_flow.height());
        ExtrusionPath templ(erPerimeter, fine_flow.mm3_per_mm(), fine_flow.width(), fine_flow.height());
        ext_templ.inset_idx = 0;
        templ.inset_idx     = 0;
        // The lines of each pass by island of the layer.
        RefinementIslands pass_islands(count);
        for (int pass = 0; pass < count; ++ pass) {
            const ExPolygons pass_zone = intersection_ex(offset(zone, run_on(pass)), shell);
            for (const ExPolygon &island : pass_slices[pass]) {
                // Concentric lines from the outline of the pass inwards, as far as the zone reaches.
                auto    extrusions = std::make_unique<ExtrusionEntityCollection>();
                coord_t distance   = fine_ext_flow.scaled_width() / 2;
                for (bool outer = true;; outer = false) {
                    const size_t num_extrusions = extrusions->entities.size();
                    for (const Polygon &loop : offset(island, - float(distance)))
                        append_refinement_loop(loop, pass_zone, outer ? ext_templ : templ, *extrusions);
                    if (extrusions->entities.size() == num_extrusions)
                        break;
                    distance += outer ? (fine_ext_flow.scaled_spacing() + fine_flow.scaled_spacing()) / 2 : fine_flow.scaled_spacing();
                }
                if (! extrusions->entities.empty())
                    pass_islands[pass].emplace_back(island_of(lslices, island.contour.points.front()), std::move(extrusions));
            }
        }
        if (ramp > 0.) {
            const ZoneDistance distance(zone);
            for (int pass = 0; pass + 1 < count; ++ pass)
                for (auto &[island, extrusions] : pass_islands[pass])
                    if (extrusions)
                        wedge_pass(*extrusions, distance, ramp, pass_height, min_height, count, pass + 1);
        }
        if (config.layer_refinement_helical.value) {
            const size_t num_passes_before = layerm.refinement_passes.size();
            make_refinement_helices(pass_islands, pass_height, layerm.refinement_passes);
            for (size_t idx = num_passes_before; idx < layerm.refinement_passes.size(); ++ idx)
                layerm.refinement_passes[idx].support = below;
        }
        for (int pass = 0; pass < count; ++ pass) {
            LayerRegion::RefinementPass refinement_pass { - (count - 1 - pass) * pass_height };
            refinement_pass.support = pass == 0 ? below : pass_slices[pass - 1];
            for (auto &[island, extrusions] : pass_islands[pass]) {
                if (! extrusions)
                    continue;
                // The open stretches of the outer wall of the top pass are spliced with the walls of the layer they
                // meet, once these are cut back.
                if (pass + 1 == count && joints) {
                    ExtrusionEntitiesPtr kept;
                    for (ExtrusionEntity *entity : extrusions->entities)
                        if (auto *path = dynamic_cast<ExtrusionPath*>(entity); path != nullptr && path->role() == erExternalPerimeter)
                            top_stretches.emplace_back(&layerm, std::make_pair(int(island), std::unique_ptr<ExtrusionPath>(path)));
                        else
                            kept.emplace_back(entity);
                    extrusions->entities = std::move(kept);
                }
                refinement_pass.islands.insert(refinement_pass.islands.end(), extrusions->entities.size(), island);
                refinement_pass.extrusions.append(std::move(extrusions->entities));
            }
            if (! refinement_pass.extrusions.entities.empty())
                layerm.refinement_passes.emplace_back(std::move(refinement_pass));
        }
    }
    if (fine_zones.empty())
        return;

    refinement_lowered     = union_(refinement_lowered);
    refinement_top_support = union_ex(refinement_top_support);
    fine_zones             = union_(fine_zones);
    for (LayerRegion *layerm : m_regions) {
        remove_extrusions_over(layerm->perimeters, fine_zones);
        for (const Ramp &ramp : ramps)
            ramp_outer_walls(layerm->perimeters, ZoneDistance(ramp.zone), ramp.length, ramp.pass_height);
        remove_extrusions_over(layerm->thin_fills, fine_zones);
        // Cutting the fine zones out of the fill areas may leave slivers, on which the gap fill of the infill, built on a
        // Voronoi diagram, fails: they are too thin to fill anyway.
        const float sliver = float(scale_(0.02));
        layerm->fill_expolygons            = opening_ex(diff_ex(layerm->fill_expolygons, fine_zones), sliver);
        layerm->fill_no_overlap_expolygons = opening_ex(diff_ex(layerm->fill_no_overlap_expolygons, fine_zones), sliver);
        // Each fill surface keeps its own type, top, bottom or internal, which the infill and the ironing go by.
        Surfaces fill_surfaces;
        for (const Surface &surface : layerm->fill_surfaces.surfaces)
            for (ExPolygon &expoly : opening_ex(diff_ex(ExPolygons{ surface.expolygon }, fine_zones), sliver))
                fill_surfaces.emplace_back(surface, std::move(expoly));
        layerm->fill_surfaces.surfaces = std::move(fill_surfaces);
    }

    // The top line of each layer runs through fine and coarse in one, as the merge pass of the prototype. The
    // stretches that meet no wall stay a pass of their own, printed at print_z with the others.
    if (! top_stretches.empty()) {
        std::vector<std::unique_ptr<ExtrusionPath>> stretches;
        for (auto &[layerm, stretch] : top_stretches)
            stretches.emplace_back(std::move(stretch.second));
        splice_top_passes(m_regions, stretches);
        for (size_t idx = 0; idx < stretches.size(); ++ idx) {
            if (! stretches[idx])
                continue;
            LayerRegion &layerm = *top_stretches[idx].first;
            auto it = std::find_if(layerm.refinement_passes.begin(), layerm.refinement_passes.end(),
                                   [](const LayerRegion::RefinementPass &pass) { return pass.turn_offsets.empty() && std::abs(pass.z_offset) < EPSILON; });
            if (it == layerm.refinement_passes.end()) {
                layerm.refinement_passes.push_back({ 0. });
                it          = std::prev(layerm.refinement_passes.end());
                it->support = refinement_top_support;
            }
            it->islands.emplace_back(top_stretches[idx].second.first);
            it->extrusions.entities.emplace_back(stretches[idx].release());
        }
    }
}

void Layer::export_region_slices_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const auto *region : m_regions)
        for (const auto &surface : region->slices.surfaces)
            bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const auto *region : m_regions)
        for (const auto &surface : region->slices.surfaces)
            svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void Layer::export_region_slices_to_svg_debug(const char *name) const
{
    static size_t idx = 0;
    this->export_region_slices_to_svg(debug_out_path("Layer-slices-%s-%d.svg", name, idx ++).c_str());
}

void Layer::export_region_fill_surfaces_to_svg(const char *path) const
{
    BoundingBox bbox;
    for (const auto *region : m_regions)
        for (const auto &surface : region->slices.surfaces)
            bbox.merge(get_extents(surface.expolygon));
    Point legend_size = export_surface_type_legend_to_svg_box_size();
    Point legend_pos(bbox.min(0), bbox.max(1));
    bbox.merge(Point(std::max(bbox.min(0) + legend_size(0), bbox.max(0)), bbox.max(1) + legend_size(1)));

    SVG svg(path, bbox);
    const float transparency = 0.5f;
    for (const auto *region : m_regions)
        for (const auto &surface : region->slices.surfaces)
            svg.draw(surface.expolygon, surface_type_to_color_name(surface.surface_type), transparency);
    export_surface_type_legend_to_svg(svg, legend_pos);
    svg.Close();
}

//BBS: method to simplify support path
void Layer::simplify_support_entity_collection(ExtrusionEntityCollection* entity_collection)
{
    for (size_t i = 0; i < entity_collection->entities.size(); i++) {
        if (ExtrusionEntityCollection* collection = dynamic_cast<ExtrusionEntityCollection*>(entity_collection->entities[i]))
            this->simplify_support_entity_collection(collection);
        else if (ExtrusionPath* path = dynamic_cast<ExtrusionPath*>(entity_collection->entities[i]))
            this->simplify_support_path(path);
        else if (ExtrusionMultiPath* multipath = dynamic_cast<ExtrusionMultiPath*>(entity_collection->entities[i]))
            this->simplify_support_multi_path(multipath);
        else if (ExtrusionLoop* loop = dynamic_cast<ExtrusionLoop*>(entity_collection->entities[i]))
            this->simplify_support_loop(loop);
        else
            throw Slic3r::InvalidArgument("Invalid extrusion entity supplied to simplify_support_entity_collection()");
    }
}
//BBS: method to simplify support path
void Layer::simplify_support_path(ExtrusionPath * path)
{
    const auto &print_config = this->object()->print()->config();
    const bool spiral_mode = print_config.spiral_mode;
    const bool enable_arc_fitting = print_config.enable_arc_fitting.value == ArcFittingMode::Enabled;
    const auto scaled_resolution = scaled<double>(print_config.resolution.value);

    if (enable_arc_fitting &&
        !spiral_mode) {
        path->simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
    } else {
        path->simplify(scaled_resolution);
    }
}
//BBS: method to simplify support path
void Layer::simplify_support_multi_path(ExtrusionMultiPath* multipath)
{
    const auto &print_config = this->object()->print()->config();
    const bool spiral_mode = print_config.spiral_mode;
    const bool enable_arc_fitting = print_config.enable_arc_fitting.value == ArcFittingMode::Enabled;
    const auto scaled_resolution = scaled<double>(print_config.resolution.value);

    for (size_t i = 0; i < multipath->paths.size(); ++i) {
        if (enable_arc_fitting &&
            !spiral_mode) {
            multipath->paths[i].simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
        } else {
            multipath->paths[i].simplify(scaled_resolution);
        }
    }
}
//BBS: method to simplify support path
void Layer::simplify_support_loop(ExtrusionLoop* loop)
{
    const auto &print_config = this->object()->print()->config();
    const bool spiral_mode = print_config.spiral_mode;
    const bool enable_arc_fitting = print_config.enable_arc_fitting.value == ArcFittingMode::Enabled;
    const auto scaled_resolution = scaled<double>(print_config.resolution.value);

    for (size_t i = 0; i < loop->paths.size(); ++i) {
        if (enable_arc_fitting &&
            !spiral_mode) {
            loop->paths[i].simplify_by_fitting_arc(SCALED_SUPPORT_RESOLUTION);
        } else {
            loop->paths[i].simplify(scaled_resolution);
        }
    }
}

// Export to "out/LayerRegion-name-%d.svg" with an increasing index with every export.
void Layer::export_region_fill_surfaces_to_svg_debug(const char *name) const
{
    static size_t idx = 0;
    this->export_region_fill_surfaces_to_svg(debug_out_path("Layer-fill_surfaces-%s-%d.svg", name, idx ++).c_str());
}

coordf_t Layer::get_sparse_infill_max_void_area()
{
    double max_void_area = 0.;
    for (auto layerm : m_regions) {
        Flow flow = layerm->flow(frInfill);
        const PrintRegionConfig &config = layerm->region().config();
        float density = config.sparse_infill_density;
        InfillPattern pattern = config.sparse_infill_pattern;
        if (density == 0.)
            return -1;
        // Orca: the adaptive TPMS infill is as sparse as its interior density.
        if (density < 100.f && config.adaptive_infill_mode != AdaptiveInfillMode::Disabled && is_adaptive_infill_pattern(pattern))
            density = std::min(density, std::max(1.f, float(config.adaptive_infill_interior_density)));

        //BBS: rough estimation and need to be optimized
        double spacing = flow.scaled_spacing() * (100 - density) / density;
        switch (pattern) {
            case ipConcentric:
            case ipSpiralInset:
            case ipRectilinear:
            case ipLine:
            case ipGyroid:
            case ipTpmsD:
            case ipTpmsFK:
            case ipAlignedRectilinear:
            case ipOctagramSpiral:
            case ipHilbertCurve:
            case ipLateralHoneycomb:
            case ip3DHoneycomb:
            case ipArchimedeanChords:
                max_void_area = std::max(max_void_area, spacing * spacing);
                break;
            case ipGrid:
            case ipLateralLattice:
            case ipHoneycomb:
            case ipLightning:
                max_void_area = std::max(max_void_area, 4.0 * spacing * spacing);
                break;
            case ipCubic:
            case ipAdaptiveCubic:
            case ipTriangles:
            case ipStars:
            case ipSupportCubic:
                max_void_area = std::max(max_void_area, 4.5 * spacing * spacing);
                break;
            default:
                max_void_area = std::max(max_void_area, spacing * spacing);
                break;
        }
    };
    return max_void_area;
}

size_t Layer::get_extruder_id(unsigned int filament_id) const
{
    return m_object->print()->get_extruder_id(filament_id);
}

BoundingBox get_extents(const LayerRegion &layer_region)
{
    BoundingBox bbox;
    if (!layer_region.slices.surfaces.empty()) {
        bbox = get_extents(layer_region.slices.surfaces.front());
        for (auto it = layer_region.slices.surfaces.cbegin() + 1; it != layer_region.slices.surfaces.cend(); ++it)
            bbox.merge(get_extents(*it));
    }
    return bbox;
}

BoundingBox get_extents(const LayerRegionPtrs &layer_regions)
{
    BoundingBox bbox;
    if (!layer_regions.empty()) {
        bbox = get_extents(*layer_regions.front());
        for (auto it = layer_regions.begin() + 1; it != layer_regions.end(); ++it)
            bbox.merge(get_extents(**it));
    }
    return bbox;
}

}
