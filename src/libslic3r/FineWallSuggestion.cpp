#include "FineWallSuggestion.hpp"

#include "AABBMesh.hpp"
#include "libslic3r.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <memory>

#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>

namespace Slic3r {

// Only a face within half a degree of flat is a top or a bottom surface: the nearly flat faces leave the widest and
// most visible steps.
static constexpr double flat_normal_z = 0.99996;

double fine_wall_step(double normal_z, double layer_height)
{
    const double nz = std::abs(normal_z);
    if (nz >= flat_normal_z)
        return 0.;
    return layer_height * nz / std::sqrt(1. - nz * nz);
}

FineWallSuggestion suggest_fine_walls(const indexed_triangle_set &its, const Transform3d &trafo, const FineWallRules &rules)
{
    const size_t       num_facets = its.indices.size();
    FineWallSuggestion out;
    out.score.assign(num_facets, 0.f);
    out.selected.assign(num_facets, false);
    if (num_facets == 0)
        return out;

    // A step of half a millimetre or more scores 1.
    constexpr double full_step   = 0.5;
    const Matrix3d   normal_trafo = trafo.linear().inverse().transpose();
    const Vec3d      up           = (trafo.linear().inverse() * Vec3d::UnitZ()).normalized();
    const double     sin_overhang = std::sin(rules.overhang_angle * PI / 180.);
    const bool       suggest      = rules.mode != FineWallMode::Paint;
    // A face is seen from outside when a ray from it outwards, straight or tilted up or to the side, leaves the part.
    const std::unique_ptr<AABBMesh> aabb = suggest && rules.visible_only ? std::make_unique<AABBMesh>(its) : nullptr;

    std::vector<unsigned char> selected(num_facets, 0);
    tbb::parallel_for(tbb::blocked_range<size_t>(0, num_facets), [&](const tbb::blocked_range<size_t> &range) {
        for (size_t idx = range.begin(); idx < range.end(); ++ idx) {
            const stl_triangle_vertex_indices &facet = its.indices[idx];
            const Vec3d a = its.vertices[facet(0)].cast<double>();
            const Vec3d b = its.vertices[facet(1)].cast<double>();
            const Vec3d c = its.vertices[facet(2)].cast<double>();
            const Vec3d local_normal = (b - a).cross(c - a);
            if (local_normal.squaredNorm() <= 0.)
                continue;
            const Vec3d  normal = (normal_trafo * local_normal).normalized();
            const double step   = fine_wall_step(normal.z(), rules.layer_height);
            out.score[idx] = float(std::min(1., step / full_step));
            if (! suggest || std::abs(normal.z()) >= flat_normal_z)
                continue;
            bool chosen = rules.mode == FineWallMode::OutwardWalls || step >= rules.min_step ||
                          (rules.overhangs && normal.z() < 0. && -normal.z() >= sin_overhang);
            if (chosen && aabb) {
                const Vec3d n      = local_normal.normalized();
                const Vec3d side   = n.cross(up).squaredNorm() > 1e-12 ? Vec3d(n.cross(up).normalized()) : Vec3d(n.unitOrthogonal());
                const Vec3d origin = (a + b + c) / 3. + n * 1e-3;
                const std::array<Vec3d, 4> rays { n, (n + up).normalized(), (n + side).normalized(), (n - side).normalized() };
                chosen = std::any_of(rays.begin(), rays.end(), [&aabb, &origin](const Vec3d &dir) {
                    return ! aabb->query_ray_hit(origin, dir).is_hit();
                });
            }
            selected[idx] = chosen;
        }
    });
    for (size_t idx = 0; idx < num_facets; ++ idx)
        out.selected[idx] = selected[idx] != 0;
    return out;
}

} // namespace Slic3r
