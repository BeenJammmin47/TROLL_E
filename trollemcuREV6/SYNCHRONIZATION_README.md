# TROLL E Controller — trollemcuREV6 (UNROUTED)

## What is in this project
- **trollemcuREV6.kicad_sch**: the user-uploaded cleaned schematic, unchanged electrically. Only KiCad project-instance names were updated from `TROLLEmcuREV3` to `trollemcuREV6`.
- **trollemcuREV6.kicad_pcb**: the previous REV5 edge-placement layout, with NO copper segments, all 64 original footprints and their locations, all 4 zones (GND front/back/In1, +3V3 In2), and 8 GND stitching vias.
- **trollemcuREV6.kicad_pro**: KiCad project settings.
- **trollemcuREV6_netlist.csv**: snapshot of schematic-pin-to-net connections; useful for auditing. For future updates, the schematic is authoritative; KiCad internally generates the live netlist when using Update PCB from Schematic.
- **SCHEMATIC_PCB_PIN_FIXES.csv**: all 80 pin/net assignments corrected in the PCB to match the provided schematic. The previous unrouted board had several connector pins reversed relative to the schematic. This is important for connector wiring: prior board labels MUST NOT be used; the REV6 PCB silkscreen legends and connector_pin_labels.csv have been corrected.
- **connector_pin_labels.csv**: schematic-derived connector pin-to-net listing.

## Ensuring Update PCB from Schematic is usable
- The KiCad project file, schematic file, and PCB file have the **same base name** and reside in this one directory.
- All 60 electrical footprints include unique schematic association paths in the form `/cb1d1180-fbb8-4f6b-ae50-e502b4a4b6fd/SYMBOL_UUID`, mapping each footprint to its exact schematic symbol UUID (not merely a reference designator). The remaining 4 mechanical mounting-hole footprints are PCB-only.
- All 266 schematic symbol-pin assignments were compared against PCB copper-pad assignments, including multi-pad USB-C shield contacts, after the adjustments. All match.
- Symbols use the included project-local legacy symbol library `TROLLEmcuREV3-cache.lib` through a portable relative `sym-lib-table`. The original Windows-only library alias was removed.
- Standard KiCad footprint libraries are resolved using the normal global KiCad installation; no custom footprints are required by this project.

## Updating a part or its footprint
1. Open **trollemcuREV6.kicad_pro** in a KiCad installation compatible with the KiCad 10 file format.
2. Edit your circuit in Schematic Editor. If changing a physical package, adjust the symbol's **Footprint** property and save.
3. In PCB Editor choose **Tools -> Update PCB from Schematic** (typically F8), inspect changes before applying. KiCad should recognize existing components by their UUID-backed paths and preserve positions for unchanged parts. New schematic components will appear attached to the cursor to be placed manually.
4. If you replace/delete symbols in the schematic (rather than only change properties), UUIDs can change, and KiCad may consider the footprint new. Review the update preview before accepting.
5. If you add a new connector, annotate the symbols and then update the PCB; add the appropriate connector pin labels to the front silkscreen.
6. Press **B** to refill copper zones after any placement/routing changes; then run ERC and DRC.

## Routing
- Zero routed signal traces. Eight original GND stitching vias remain; four ground/power copper zones are specified but not filled. Signal routes and additional stitching vias are yours to place.
- Pressing **V** while routing a signal produces a via on **that signal's net**, not GND. Assign ground-stitching vias to GND deliberately.
- The existing board outline is 130 x 90 mm and the original 4 M3 mounting holes and 22 J connectors have not been moved from the REV5 edge layout.

## Limitations
- I verified the textual KiCad data, its symbol-to-footprint mapping, and pad/net consistency but **could not execute KiCad's native Update PCB, ERC, or DRC in this environment**. Verify those steps in your installed KiCad before fabrication.
- The footprint of the 3.3 V regulator, USB-C trace rules, signal integrity, voltage levels, and safety/emergency-stop circuit still require normal engineering review.
