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

With `layer_refinement_scarf_length` above 0 (2 mm by default), the joint is the
scarf of the prototype (`scripts/layer_combination_coupons.py`):

- The top of the layer is one line. The open stretches of the top pass are
  spliced with the walls of the layer they meet into one loop or line
  (`splice_top_passes()`), so there is no start or end at the joint.
- Over the ramp, the wall of the layer rises from the height of a pass to its
  own, linearly where it runs into the fine zone and as u^0.8 with a bump of
  up to 10 % where it leaves it, to pre-pressurise the nozzle.
- Under it, the lower passes run on as a wedge: pass k of N ends
  `L · ((N − k) − h_min / h) / (N − 1)` past the fine zone, and its top follows
  `min(k·h, max((k − 1)·h + h_min, (N − 1)·h·(1 − u / L)))`, printed as a Z
  contoured path whose flow follows its height. The wedge and the ramp add up
  to the height of the layer everywhere.

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

Where the top pass lies further in than the layer, the passes leave the top of
the layer under `print_z`. `Layer::refinement_lowered` records these areas, and
the ironing leaves them out, as there is no top surface there to iron.

## Data

`LayerRegion::refinement_passes` holds the passes, lowest first, each with its
offset under `print_z`, the top one at 0. None of them is part of
`LayerRegion::perimeters`: the seams of their loops are looked up by position on
the walls of the layer.

All lines of a pass carry wall index 0, so all of them are printed with the
outer wall filament. Each line records the island of the layer it belongs to.
A helix is stored as a pass whose extrusions are its turns, each with its own
offset under `print_z`; its top turn replaces the top pass of that island.

## G-code

The passes are not layers. `GCode::process_layer()` prints the passes of an
object at the start of that object's extrusions on the layer, lowest first over
all regions and the top one last, by lowering the nominal Z the way the
mixed-colour sub-layers do, and then prints the layer as usual. Nothing of the
layer stands next to a pass when it is printed, and the fine walls already stand
at `print_z` when the thick ones are printed beside them. Within a pass the lines
go nearest first, and an open line starts from its end nearer to the nozzle, so
that the passes of a stretch go back and forth.

With `layer_refinement_chain_passes`, the passes of the regions with the option
are printed after the others, island by island, all passes of one island lowest
first before the next island.
Only the first pass of an island starts with a lift; the next pass starts at the
seam where the last one ended. This saves the travels between the islands for
every pass. The nozzle goes down once per island, onto an island where nothing of
the layer is printed yet.

With `layer_refinement_z_from_above` (on by default), a pass printed above the
one before is reached from 0.1 mm above it, as a layer is, so that play in the Z
axis does not change the heights of the passes against each other.
`layer_refinement_fan_speed` gives the passes a fan speed of their own: the
G-code marks them with `;_FINE_WALL_FAN_START S<speed>` and `;_FINE_WALL_FAN_END`,
and the cooling buffer applies it where no overhang or internal bridge fan speed
is asked for, as these take precedence. `layer_refinement_wall_overlap` scales how far the passes reach into the inner
wall, which is printed after them and presses against them.

A travel from a fine wall shorter than `layer_refinement_travel_without_retraction`
(5 mm by default) goes without a retraction, a wipe and a Z hop: on thin passes,
the restart after each retraction shows as a seam.

A wall thinner than a refined layer, which is a pass or the wall of the layer
thinned over a scarf, is printed as a fine wall whatever region it belongs to: the
top pass is spliced into the wall of the region it meets, which is not refined
itself. It is printed with `layer_refinement_speed` and
`layer_refinement_acceleration`, unless they overhang. A fine wall gets the
overhang speed and fan of the wall of the layer it replaces: each of its points is
measured from the nearest point of the outline of its layer against the layer below
(`FineWallBoundaries`). The passes of a layer and the turns of a helix lie at
slightly different places, which would each tip the speed and fan their own way;
measured from the outline, all of them print alike over the same stretch of it.
At 0 the speed and acceleration follow the outer and inner wall
values; the settings page then shows the outer wall value in their fields, while
the stored value stays 0 until it is edited.

Consequences:

- no layer change, layer-change G-code, timelapse or layer number for a pass;
- the cooling logic sees a layer and its passes as one layer;
- the prime tower is not affected, as no layer and no tool is added.

Tool ordering registers the outer wall filament for a layer region that holds
passes even when it has no other walls, and the object is then visited with that
filament although none of its islands has extrusions for it.

A painted modifier that sets a fine wall layer height refines walls only: unlike
other painted modifiers, its paint on faces looking up or down is not carried
through the top and bottom shells.

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
- Supports are generated from the layers, not from the passes.
- With several filaments on one object, a filament printed before the outer
  wall filament may lay thick walls next to where the lower passes follow.
- A fine zone that is not a whole number of lines deep leaves up to half a line
  of gap or overlap at the core.
- A flat top or bottom inside a layer, where the outline of one pass lies more
  than 2 mm past that of the next, is left to the top or bottom infill of the
  layer: only the outer wall along it is refined, so that surface keeps the
  layer height.
- The outlines of the passes are not changed by "make overhangs printable".
