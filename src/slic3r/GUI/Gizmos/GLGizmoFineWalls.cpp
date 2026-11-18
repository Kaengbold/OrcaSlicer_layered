#include "GLGizmoFineWalls.hpp"

#include "libslic3r/Color.hpp"
#include "libslic3r/FineWallSuggestion.hpp"
#include "libslic3r/Model.hpp"
#include "libslic3r/Point.hpp"
#include "libslic3r/PresetBundle.hpp"
#include "libslic3r/PrintConfig.hpp"
#include "libslic3r/TriangleSelector.hpp"

#include "slic3r/GUI/3DScene.hpp"
#include "slic3r/GUI/Event.hpp"
#include "slic3r/GUI/GLCanvas3D.hpp"
#include "slic3r/GUI/GUI.hpp"
#include "slic3r/GUI/GUI_App.hpp"
#include "slic3r/GUI/GUI_ObjectList.hpp"
#include "slic3r/GUI/I18N.hpp"
#include "slic3r/GUI/ImGuiWrapper.hpp"
#include "slic3r/GUI/Plater.hpp"
#include "slic3r/GUI/Selection.hpp"
#include "slic3r/GUI/Gizmos/GLGizmoPainterBase.hpp"
#include "slic3r/GUI/Gizmos/GLGizmosCommon.hpp"
#include "slic3r/Utils/UndoRedo.hpp"

#include <algorithm>
#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <glad/gl.h>
#include <imgui.h>
#include <wx/busycursor.h>

namespace Slic3r::GUI {

// Heat map colours: faces not suggested, then from a small to a large gain. A function, as GLVolume::NEUTRAL_COLOR is
// a static of another translation unit.
static std::vector<ColorRGBA> heat_map_colors()
{
    return { GLVolume::NEUTRAL_COLOR,
             ColorRGBA(0.20f, 0.40f, 1.00f, 1.f),
             ColorRGBA(0.20f, 0.80f, 0.90f, 1.f),
             ColorRGBA(0.30f, 0.85f, 0.30f, 1.f),
             ColorRGBA(1.00f, 0.85f, 0.20f, 1.f),
             ColorRGBA(1.00f, 0.30f, 0.20f, 1.f) };
}

GLGizmoFineWalls::GLGizmoFineWalls(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id)
    : GLGizmoPaintedModifier(parent, icon_filename, sprite_id)
{
}

bool GLGizmoFineWalls::is_fine_wall_modifier(const ModelVolume &volume)
{
    return volume.is_painted_modifier() && volume.config.has("layer_refinement_height") &&
           volume.config.opt_float("layer_refinement_height") > 0.;
}

std::string GLGizmoFineWalls::on_get_name() const
{
    return _u8L("Fine walls");
}

bool GLGizmoFineWalls::on_init()
{
    GLGizmoPaintedModifier::on_init();
    m_desc["mode"]             = _L("Suggest");
    m_desc["mode_paint"]       = _L("Nothing, paint only");
    m_desc["mode_outward"]     = _L("All outward-facing walls");
    m_desc["mode_slope"]       = _L("Walls with visible layer steps");
    m_desc["pass_height"]      = _L("Fine wall layer height");
    m_desc["min_step"]         = _L("Minimum step");
    m_desc["min_step_tooltip"] = _L("Suggests the walls where a layer leaves a step at least this wide on the surface, as on slopes and domes.");
    m_desc["overhangs"]        = _L("Overhangs");
    m_desc["overhang_angle"]   = _L("Overhang angle");
    m_desc["visible_only"]     = _L("Only faces seen from outside");
    m_desc["heat_map"]         = _L("Heat map");
    m_desc["heat_map_tooltip"] = _L("Colours the suggested faces from blue, where fine walls gain little, to red, where they gain the most. "
                                    "Grey faces are not suggested. The heat map shows the suggestion, not the painted area.");
    m_desc["apply"]            = _L("Apply suggestion");
    m_desc["apply_tooltip"]    = _L("Replaces the painted fine wall area of each part with the suggestion. You can edit it with the brush afterwards.");
    return true;
}

double GLGizmoFineWalls::layer_height(const ModelObject &mo) const
{
    if (mo.config.has("layer_height"))
        return mo.config.opt_float("layer_height");
    return wxGetApp().preset_bundle->prints.get_edited_preset().config.opt_float("layer_height");
}

void GLGizmoFineWalls::make_fine_wall_modifier(ModelVolume &painted_modifier) const
{
    painted_modifier.name = _u8L("Fine walls");
    painted_modifier.config.set("layer_refinement_height", double(m_pass_height));
}

void GLGizmoFineWalls::update_from_model_object(bool first_update)
{
    if (const ModelObject *mo = m_c->selection_info()->model_object(); first_update && mo != nullptr) {
        // Opened from the toolbar: keep the fine wall modifier chosen in the object list, else continue with the last
        // one of the object, or start a new one.
        const ModelVolume *current = target(*mo);
        if (current == nullptr || ! is_fine_wall_modifier(*current)) {
            m_target_id = ObjectID();
            auto it = std::find_if(mo->volumes.rbegin(), mo->volumes.rend(), [](const ModelVolume *v) { return is_fine_wall_modifier(*v); });
            if (it != mo->volumes.rend())
                m_target_id = (*it)->id();
        }
    }
    GLGizmoPaintedModifier::update_from_model_object(false);
    m_heat_map_dirty = true;
}

void GLGizmoFineWalls::update_model_object()
{
    const ObjectID target_before = m_target_id;
    GLGizmoPaintedModifier::update_model_object();
    ModelObject *mo = m_c->selection_info()->model_object();
    if (mo == nullptr || m_target_id == target_before)
        return;
    // The first stroke created a painted modifier: it sets the fine wall layer height.
    if (ModelVolume *painted_modifier = target(*mo); painted_modifier != nullptr && ! is_fine_wall_modifier(*painted_modifier)) {
        make_fine_wall_modifier(*painted_modifier);
        const ModelObjectPtrs &objects = wxGetApp().model().objects;
        wxGetApp().obj_list()->changed_object(int(std::find(objects.begin(), objects.end(), mo) - objects.begin()));
        wxGetApp().obj_list()->update_and_show_object_settings_item();
    }
}

void GLGizmoFineWalls::update_heat_map()
{
    const ModelObject *mo = m_c->selection_info()->model_object();
    if (! m_heat_map_dirty || mo == nullptr)
        return;
    m_heat_map_dirty = false;

    wxBusyCursor wait;
    m_rules.layer_height   = layer_height(*mo);
    m_rules.min_step       = m_min_step;
    m_rules.overhang_angle = m_overhang_angle;
    const int         instance_idx = std::max(0, m_parent.get_selection().get_instance_idx());
    const Transform3d instance     = mo->instances[instance_idx]->get_transformation().get_matrix();

    m_suggestions.clear();
    m_heat_selectors.clear();
    const std::vector<ColorRGBA> colors = heat_map_colors();
    const int                    num_buckets = int(colors.size()) - 1;
    for (const ModelVolume *mv : mo->volumes) {
        if (! mv->is_model_part())
            continue;
        const TriangleMesh &mesh = mv->mesh();
        m_suggestions.emplace_back(suggest_fine_walls(mesh.its, instance * mv->get_matrix(), m_rules));
        const FineWallSuggestion &suggestion = m_suggestions.back();

        auto selector = std::make_unique<TriangleSelectorPatch>(mesh, colors);
        for (size_t idx = 0; idx < suggestion.score.size(); ++ idx) {
            // Without a suggestion, the heat map shows the gain of every face.
            if (! suggestion.selected[idx] && m_rules.mode != FineWallMode::Paint)
                continue;
            if (m_rules.mode == FineWallMode::Paint && suggestion.score[idx] <= 0.f)
                continue;
            const int bucket = 1 + std::min(num_buckets - 1, int(suggestion.score[idx] * float(num_buckets)));
            selector->set_facet(int(idx), static_cast<EnforcerBlockerType>(bucket));
        }
        selector->request_update_render_data();
        m_heat_selectors.emplace_back(std::move(selector));
    }
}

void GLGizmoFineWalls::render_painter_gizmo()
{
    if (! m_show_heat_map) {
        GLGizmoPaintedModifier::render_painter_gizmo();
        return;
    }
    update_heat_map();
    if (m_heat_selectors.size() != m_triangle_selectors.size()) {
        GLGizmoPaintedModifier::render_painter_gizmo();
        return;
    }

    const Selection &selection = m_parent.get_selection();
    glsafe(::glEnable(GL_BLEND));
    glsafe(::glEnable(GL_DEPTH_TEST));
    // The heat map is drawn by the base renderer with the heat map selectors in place of the paint ones.
    std::swap(m_triangle_selectors, m_heat_selectors);
    render_triangles(selection);
    std::swap(m_triangle_selectors, m_heat_selectors);
    m_c->object_clipper()->render_cut();
    m_c->instances_hider()->render_cut();
    render_cursor();
    glsafe(::glDisable(GL_BLEND));
}

void GLGizmoFineWalls::apply_suggestion()
{
    ModelObject *mo = m_c->selection_info()->model_object();
    if (mo == nullptr)
        return;
    m_heat_map_dirty = true;
    update_heat_map();

    std::vector<ModelVolume *> parts;
    for (ModelVolume *mv : mo->volumes)
        if (mv->is_model_part())
            parts.push_back(mv);
    if (parts.size() != m_suggestions.size())
        return;

    Plater::TakeSnapshot   snapshot(wxGetApp().plater(), _u8L("Apply fine wall suggestion"), UndoRedo::SnapshotType::GizmoAction);
    const ModelObjectPtrs &objects = wxGetApp().model().objects;
    const int              obj_idx = int(std::find(objects.begin(), objects.end(), mo) - objects.begin());
    ModelVolume           *last    = nullptr;
    for (size_t part_idx = 0; part_idx < parts.size(); ++ part_idx) {
        const ModelVolume        &part       = *parts[part_idx];
        const FineWallSuggestion &suggestion = m_suggestions[part_idx];
        TriangleSelector          selector(part.mesh());
        bool                      any = false;
        for (size_t idx = 0; idx < suggestion.selected.size(); ++ idx)
            if (suggestion.selected[idx]) {
                selector.set_facet(int(idx), EnforcerBlockerType::ENFORCER);
                any = true;
            }
        auto it = std::find_if(mo->volumes.begin(), mo->volumes.end(), [mo, &part](const ModelVolume *v) {
            return is_fine_wall_modifier(*v) && mo->painted_modifier_host(*v) == &part;
        });
        if (it != mo->volumes.end()) {
            (*it)->painted_modifier_facets.set(selector);
            last = *it;
        } else if (any) {
            last = wxGetApp().obj_list()->add_painted_modifier(obj_idx, part, &selector);
            make_fine_wall_modifier(*last);
        }
    }
    wxGetApp().obj_list()->changed_object(obj_idx);
    if (last != nullptr) {
        m_target_id = last->id();
        wxGetApp().obj_list()->select_item(ObjectVolumeID{mo, last});
    }
    GLGizmoPaintedModifier::update_from_model_object(false);
    // Show the paint, which the heat map hides, so that it can be edited and erased with the brush.
    m_show_heat_map = false;
    m_parent.post_event(SimpleEvent(EVT_GLCANVAS_SCHEDULE_BACKGROUND_PROCESS));
    m_parent.set_as_dirty();
}

float GLGizmoFineWalls::extra_labels_width() const
{
    float width = 0.f;
    for (const char *key : {"mode", "pass_height", "min_step", "overhang_angle"})
        width = std::max(width, m_imgui->calc_text_size(m_desc.at(key)).x);
    return width;
}

void GLGizmoFineWalls::on_render_extra_input(float sliders_left_width, float sliders_width)
{
    ImGui::Separator();

    const float slider_icon_width = m_imgui->get_slider_icon_size().x;
    const float space_size        = m_imgui->get_style_scaling() * 8;
    const float drag_left_width   = ImGui::GetStyle().WindowPadding.x + sliders_width - space_size;
    // A slider with the value to type in next to it, as the brush size has.
    const auto slider = [&](const char *id, float *value, float min, float max, const char *format) {
        ImGui::SameLine(sliders_left_width);
        ImGui::PushItemWidth(sliders_width);
        bool changed = m_imgui->bbl_slider_float_style((std::string("##") + id).c_str(), value, min, max, format, 1.0f, true);
        ImGui::SameLine(drag_left_width + sliders_left_width);
        ImGui::PushItemWidth(1.5f * slider_icon_width);
        changed |= ImGui::BBLDragFloat((std::string("##") + id + "_input").c_str(), value, 0.05f, min, max, format);
        *value = std::clamp(*value, min, max);
        return changed;
    };

    std::vector<std::string> modes = {into_u8(m_desc["mode_paint"]), into_u8(m_desc["mode_outward"]), into_u8(m_desc["mode_slope"])};
    float                    combo_width = sliders_width + slider_icon_width * 1.5f + space_size;
    for (const std::string &mode : modes)
        combo_width = std::max(combo_width, ImGui::CalcTextSize(mode.c_str()).x + ImGui::GetFrameHeight() * 2.f);
    int mode = int(m_rules.mode);
    ImGuiWrapper::push_combo_style(m_parent.get_scale());
    if (m_imgui->combo(m_desc["mode"], modes, mode, 0, sliders_left_width, combo_width)) {
        m_rules.mode     = static_cast<FineWallMode>(mode);
        m_heat_map_dirty = true;
    }
    ImGuiWrapper::pop_combo_style();

    ImGui::AlignTextToFramePadding();
    m_imgui->text(m_desc["pass_height"]);
    slider("fine_wall_pass_height", &m_pass_height, 0.02f, 0.2f, "%.3f");

    if (m_rules.mode == FineWallMode::Slope) {
        ImGui::AlignTextToFramePadding();
        m_imgui->text(m_desc["min_step"]);
        if (ImGui::IsItemHovered())
            m_imgui->tooltip(m_desc["min_step_tooltip"], ImGui::GetFontSize() * 20.0f);
        if (slider("fine_wall_min_step", &m_min_step, 0.f, 1.f, "%.2f"))
            m_heat_map_dirty = true;

        if (m_imgui->bbl_checkbox(m_desc["overhangs"], m_rules.overhangs))
            m_heat_map_dirty = true;
        if (m_rules.overhangs) {
            ImGui::AlignTextToFramePadding();
            m_imgui->text(m_desc["overhang_angle"]);
            const std::string format_str = std::string("%.f") + I18N::translate_utf8("°", "Face angle threshold,"
                                                                                          "placed after the number with no whitespace in between.");
            if (slider("fine_wall_overhang_angle", &m_overhang_angle, 0.f, 90.f, format_str.data()))
                m_heat_map_dirty = true;
        }
    }
    if (m_rules.mode != FineWallMode::Paint && m_imgui->bbl_checkbox(m_desc["visible_only"], m_rules.visible_only))
        m_heat_map_dirty = true;

    m_imgui->bbl_checkbox(m_desc["heat_map"], m_show_heat_map);
    if (ImGui::IsItemHovered())
        m_imgui->tooltip(m_desc["heat_map_tooltip"], ImGui::GetFontSize() * 20.0f);

    m_imgui->disabled_begin(m_rules.mode == FineWallMode::Paint);
    if (m_imgui->button(m_desc["apply"]))
        apply_suggestion();
    if (ImGui::IsItemHovered())
        m_imgui->tooltip(m_desc["apply_tooltip"], ImGui::GetFontSize() * 20.0f);
    m_imgui->disabled_end();
}

} // namespace Slic3r::GUI
