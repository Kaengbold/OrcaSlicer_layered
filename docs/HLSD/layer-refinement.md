# Layer refinement — High Level Design

## Purpose and scope

Layer refinement prints the outer surface of a part in several thin passes per
layer, while the walls and the infill behind it keep the layer height. A layer
of 0.2 mm refined to 0.05 mm gets four passes, each following the outline of the
model at its own height, so the surface shows the steps of a 0.05 mm print for
little more than the print time of a 0.2 mm one.

It is switched on by `layer_refinement_height`, the target height of one pass.
This is a region setting, so it can be set for a whole print, an object, a part,
a modifier volume or a painted modifier (see
[painted-modifiers.md](painted-modifiers.md)). The default of 0 disables it, and
no code path of the feature runs.

A whole object or a whole height range that should be printed fine, infill
included, is the job of the object layer height, the height range modifiers and
the variable layer height. Refinement only ever produces a solid fine skin.

## Passes

A layer of height `H` in a region with a pass height `h` is printed in
`N = round(H / h)` passes of height `H / N` (`layer_refinement_pass_count()`).
A pass is never thinner than 0.02 mm, and `N = 1` leaves the layer as it is.
With a variable layer height, `N` is chosen per layer.

The first layer, the layers shrunk by the elephant foot compensation, spiral
vase prints and belt printers are not refined. A region with fuzzy skin or
without walls keeps its layer height.

## Geometry

The layers of the object are sliced, classified and filled as always.
Refinement runs at the end of the perimeter step
(`PrintObject::apply_layer_refinement()`), on finished walls:

1. The printable volumes are sliced once more at the middle of each pass
   (`PrintObject::slice_printable_at()`), with the XY size compensation of the
   object. These are the pass outlines.
2. `Layer::apply_refinement()` works out the *fine zone* of the layer: what any
   pass outline covers, less the *core*.
3. The walls, gap fill and fill area of the layer are removed from the fine
   zone, and each pass fills its part of the zone with concentric lines from its
   own outline inwards.

The core is where the layer keeps its own extrusions. It starts behind the outer
wall of the layer, so the inner walls keep the layer height. Only where the outer
wall of a pass would run over an inner wall, by more than a quarter of a wall
width, is that stretch cut out of the core. The decision is local: a slope on one
side of a layer leaves the inner walls of its other sides alone. This one rule
covers the cases:

| Geometry | Fine zone |
| --- | --- |
| Upright wall | The outlines coincide: the passes replace the outer wall only. |
| Outline moving outwards (the passes lie further out the lower they are) | The zone widens outwards and the passes fill the steps in front of the inner wall. |
| Outline moving inwards (an overhang, a flaring bow) | The stretch of the inner walls the lower passes run over is cut out and printed by the passes; the rest of the inner walls keep the layer height. |
| Feature too thin for a core | The passes print all of it, solid. |

Because the core lies inside every pass outline, no thick line of the layer can
stick out of the fine surface.

With `layer_refinement_min_step` above 0, a layer is refined only where it
leaves a visible step: where the outlines of its passes lie at least that far
apart, as on slopes and domes, through the depth of the walls there. Gaps of
less than a millimetre between such stretches are refined too, so that joints
do not pile up on a curved outline. Upright walls keep the layer height, which
saves the time of printing their outer wall several times for no visible gain.
The ends of a refined stretch are ordinary joints.

A line of a pass takes the place of a wall of the layer: it is as wide as that
wall's spacing plus the mean overlap of a thin and a thick line, which keeps the
outer face on the outline and the inner face against the first wall of the core.

Regions refined into the same number of passes are refined as one, with the
flows of the first of them. The pass count of a layer is decided per region, but
the depth of the core is decided per layer for all of these regions together.

### Joints

Where a refined region ends on a wall that carries on at the layer height, the
fine lines and the thick wall meet in a joint. The fine lines can run
`layer_refinement_joint_overlap` (0 by default) into the thick wall, to fill the
gap its rounded end leaves next to the thin passes.

With `layer_refinement_scarf_length` above 0 (0.2 mm by default), the joint is a
stepped scarf. The
scarf length is divided into one step fewer than there are passes. The lower
passes run further into the thick wall, one more step for each pass down, and
over each step the thick wall is extruded with the share of the layer height
that the passes under it leave free: over the first step all passes but the top
one lie under it and it keeps one pass height, over the last step only the
lowest pass does. The extruded volume stays that of the unrefined wall, and the
joint is spread over the scarf length instead of one vertical face.

### Helices

With `layer_refinement_helical`, the passes of an island whose fine wall is a
single closed outer loop in every pass are printed as one helix instead of
separate loops: a flat turn on the layer below, a turn that ramps the flow up as
it rises off it, one turn for each further pass, and a flat turn at `print_z`
that ramps the flow down. The bottom and the top of the layer stay flat and the
volume is that of the passes, but the passes have no seams. Each turn follows
the outline of its pass, opened at the point closest to the back of the top one.
The helix is one more loop of travel than the passes, so it is about quality,
not time. Islands with wider fine zones or joints keep flat passes.

## Data

`LayerRegion::refinement_passes` holds the passes below the top one, lowest
first, each with its offset under `print_z`. The top pass is printed at
`print_z` and is stored as one more collection in `LayerRegion::perimeters`, so
the seam placer plans its seams on the fine outer wall and the lower passes,
looked up by position, get the same seam.

All lines of a pass carry wall index 0, so all of them are printed with the
outer wall filament. Each line records the island of the layer it belongs to.
A helix is stored as a pass whose extrusions are its turns, each with its own
offset under `print_z`; its top turn replaces the top pass of that island.

## G-code

The passes are not layers. `GCode::process_layer()` prints the lower passes of
an object at the start of that object's extrusions on the layer, lowest first
over all regions, by lowering the nominal Z the way the mixed-colour sub-layers
do, and then prints the layer as usual. Nothing of the layer stands next to a
pass when it is printed, and the fine walls already stand at `print_z` when the
thick ones are printed beside them.

With `layer_refinement_chain_passes`, the passes of the regions with the option
are printed after the others, island by island, all passes of one island lowest
first before the next island.
Only the first pass of an island starts with a lift; the next pass starts at the
seam where the last one ended. This saves the travels between the islands for
every pass. The nozzle goes down once per island, onto an island where nothing of
the layer is printed yet.

A wall thinner than the layer in a refined region, which is a pass or the wall
of the layer thinned over a scarf, is printed with `layer_refinement_speed` and
`layer_refinement_acceleration`. At 0 they follow the outer and inner wall
values; the settings page then shows the outer wall value in their fields, while
the stored value stays 0 until it is edited.

Consequences:

- no layer change, layer-change G-code, timelapse or layer number for a pass;
- the cooling logic sees a layer and its passes as one layer;
- the prime tower is not affected, as no layer and no tool is added.

Tool ordering registers the outer wall filament for a layer region that holds
passes even when it has no other walls, and the object is then visited with that
filament although none of its islands has extrusions for it.

## Suggestions

The fine walls tool (`GLGizmoFineWalls`) in the 3D view toolbar chooses where
fine walls go. It is the painted modifier tool (see
[painted-modifiers.md](painted-modifiers.md)) for painted modifiers that set
`layer_refinement_height`: the first stroke creates one on the part it paints.

`suggest_fine_walls()` (`FineWallSuggestion.cpp`) scores every facet of a part by
the horizontal step a layer leaves on it, `H · |n_z| / sqrt(1 − n_z²)`, as in
adaptive layer height; only faces within half a degree of flat, which are printed
as top and bottom surfaces, score 0. Nearly flat faces leave the widest steps and
score highest. It selects facets by mode:

| Mode | Selected |
| --- | --- |
| Paint only | Nothing; the heat map shows the score of every face. |
| All outward-facing walls | Every wall facet. |
| Walls with visible layer steps | Wall facets stepping at least the minimum step, and, optionally, overhangs past an angle. |

With "only faces seen from outside", a facet is kept only when at least one of
four rays from it (along its normal, tilted up, and tilted to either side)
leaves the part, which drops cavities and the insides of holes.

The heat map colours the selected facets from blue (small gain) to red (large
gain) and leaves the others grey. "Apply suggestion" replaces the paint of the
fine wall modifier of each part with the selection, creating the modifier where
there is none; the result is ordinary paint, edited with the brush and saved in
the project like any painted modifier. In the G-code preview, the layer height
view shows where the passes went.

## Known limitations

- The feature is experimental until the strength of the joints is measured.
- Fine lines are not classified as overhangs and get no overhang speed or fan.
- Supports are generated from the layers, not from the passes.
- With several filaments on one object, a filament printed before the outer
  wall filament may lay thick walls next to where the lower passes follow.
- A fine zone that is not a whole number of lines deep leaves up to half a line
  of gap or overlap at the core.
- A flat top between two layers is filled with concentric lines.
- The outlines of the passes are not changed by "make overhangs printable".
