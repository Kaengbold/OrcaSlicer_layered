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

The core is where the layer keeps its own extrusions. It starts behind the first
wall of the layer that leaves every pass room for its outer wall, with a quarter
of a wall width of tolerance. This one rule covers the cases:

| Geometry | Fine zone |
| --- | --- |
| Upright wall | The outlines coincide, the core starts behind the outer wall: the passes replace the outer wall only. |
| Slope, dome | The outlines spread apart, the core retreats behind further walls and the passes fill the steps. |
| Feature too thin for a core | The passes print all of it, solid. |
| No wall of the layer leaves room | All walls are replaced, and the core is cut back to what every pass covers. |

Because the core lies inside every pass outline, no thick line of the layer can
stick out of the fine surface.

A line of a pass takes the place of a wall of the layer: it is as wide as that
wall's spacing plus the mean overlap of a thin and a thick line, which keeps the
outer face on the outline and the inner face against the first wall of the core.

Regions refined into the same number of passes are refined as one, with the
flows of the first of them. The pass count of a layer is decided per region, but
the depth of the core is decided per layer for all of these regions together.

### Joints

Where a refined region ends on a wall that carries on at the layer height, the
fine lines and the thick wall meet in a joint. The fine lines run
`layer_refinement_joint_overlap` (0.3 mm by default) into the thick wall, to fill
the gap its rounded end leaves next to the thin passes.

With `layer_refinement_scarf_length` above 0, the joint is a stepped scarf. The
scarf length is divided into one step fewer than there are passes. The lower
passes run further into the thick wall, one more step for each pass down, and
over each step the thick wall is extruded with the share of the layer height
that the passes under it leave free: over the first step all passes but the top
one lie under it and it keeps one pass height, over the last step only the
lowest pass does. The extruded volume stays that of the unrefined wall, and the
joint is spread over the scarf length instead of one vertical face.

## Data

`LayerRegion::refinement_passes` holds the passes below the top one, lowest
first, each with its offset under `print_z`. The top pass is printed at
`print_z` and is stored as one more collection in `LayerRegion::perimeters`, so
the seam placer plans its seams on the fine outer wall and the lower passes,
looked up by position, get the same seam.

All lines of a pass carry wall index 0, so all of them are printed with the
outer wall filament.

## G-code

The passes are not layers. `GCode::process_layer()` prints the lower passes of
an object at the start of that object's extrusions on the layer, lowest first
over all regions, by lowering the nominal Z the way the mixed-colour sub-layers
do, and then prints the layer as usual. Nothing of the layer stands next to a
pass when it is printed, and the fine walls already stand at `print_z` when the
thick ones are printed beside them.

Consequences:

- no layer change, layer-change G-code, timelapse or layer number for a pass;
- the cooling logic sees a layer and its passes as one layer;
- the prime tower is not affected, as no layer and no tool is added.

Tool ordering registers the outer wall filament for a layer region that holds
passes even when it has no other walls, and the object is then visited with that
filament although none of its islands has extrusions for it.

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
