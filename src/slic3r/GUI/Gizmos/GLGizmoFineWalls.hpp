#pragma once

#include "GLGizmoPaintedModifier.hpp"

#include "libslic3r/FineWallSuggestion.hpp"
#include "slic3r/GUI/I18N.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

#include <wx/string.h>

namespace Slic3r {
class ModelObject;
class ModelVolume;
} // namespace Slic3r

namespace Slic3r::GUI {

class GLCanvas3D;
class TriangleSelectorGUI;

// Orca: Chooses where fine walls go, see the "Suggestions" section of docs/HLSD/layer-refinement.md. It paints painted
// modifiers that set a fine wall layer height, and shows a heat map of where fine walls gain the most and which faces
// the current rules suggest.
class GLGizmoFineWalls : public GLGizmoPaintedModifier
{
public:
    GLGizmoFineWalls(GLCanvas3D &parent, const std::string &icon_filename, unsigned int sprite_id);

    void render_painter_gizmo() override;

    // Whether `volume` is a painted modifier made by this tool: one that sets a fine wall layer height.
    static bool is_fine_wall_modifier(const ModelVolume &volume);

protected:
    std::string on_get_name() const override;
    void        on_render_extra_input(float sliders_left_width, float sliders_width) override;
    float       extra_labels_width() const override;

    std::string get_gizmo_entering_text() const override { return _u8L("Entering fine walls"); }
    std::string get_gizmo_leaving_text() const override { return _u8L("Leaving fine walls"); }
    std::string get_action_snapshot_name() const override { return _u8L("Fine walls editing"); }

private:
    bool on_init() override;
    void update_model_object() override;
    void update_from_model_object(bool first_update) override;

    // Recomputes the suggestion and the heat map when the rules or the object changed.
    void update_heat_map();
    // Paints the suggestion into the fine wall modifier of each part, creating it where there is none.
    void apply_suggestion();
    // Turns a new painted modifier into a fine wall one.
    void make_fine_wall_modifier(ModelVolume &painted_modifier) const;
    double layer_height(const ModelObject &mo) const;

    FineWallRules m_rules;
    float         m_pass_height { 0.05f };
    float         m_min_step { 0.1f };
    float         m_overhang_angle { 45.f };
    bool          m_show_heat_map { true };
    // The heat map dirty since the rules or the object changed.
    bool          m_heat_map_dirty { true };

    std::vector<FineWallSuggestion>                   m_suggestions;
    std::vector<std::unique_ptr<TriangleSelectorGUI>> m_heat_selectors;
    std::map<std::string, wxString>                   m_desc;
};

} // namespace Slic3r::GUI
