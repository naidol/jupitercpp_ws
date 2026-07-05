// ir_receiver_shroud.scad — field-of-view sleeve for the Jupiter dock IR receivers
// (VS1838B-class 38kHz TSOP on the "IR Receiver" breakout, vertical 3-leg mount).
//
// PURPOSE: clip the TSOP's ~+/-45deg acceptance down to a narrow forward cone so it
// sees the DIRECT dock beam and rejects reflected IR (the bench-flood problem).
//
// PRINT NOTES (both matter — learned the hard way):
//   * BLACK PLA/PETG, >=3 perimeters / wall >= 2mm so it is IR-OPAQUE. Foam-test a
//     scrap against the beacon before trusting it — thin walls leak 940nm.
//   * Keep the BORE MATTE — do NOT sand/smooth it. A shiny bore light-pipes off-axis
//     rays onto the sensor and defeats the whole shroud.
//
// This is the BARREL (universal). The mount to your exact board/bracket needs 3
// measurements — see NOTES at the bottom; give them to me and I'll add the mount.

/* ================= MEASURE THESE ================= */
dome_dia    = 6.0;   // widest part of the TSOP dome/package to clear (mm)
clearance   = 1.0;   // radial air gap around the dome (mm)
protrusion  = 18;    // barrel length PAST the dome front  <-- this sets the cone angle
back_len    = 5;     // barrel length BEHIND the dome front (surrounds the package)
wall        = 2.4;   // wall thickness (>= 2mm for opacity + stiffness)

/* ================= FOV READOUT ================= */
bore_dia = dome_dia + 2*clearance;
echo(str(">>> bore = ", bore_dia, " mm   approx FOV half-angle = ",
         atan((bore_dia/2)/protrusion), " deg   (shorter protrusion = wider)"));

$fn = 96;

// --- the barrel ---
difference() {
    cylinder(d = bore_dia + 2*wall, h = back_len + protrusion);
    translate([0,0,-0.5]) cylinder(d = bore_dia, h = back_len + protrusion + 1);
}

// ================= MOUNTING (needs your numbers) =================
// Pick one; send me the measurements and I'll add it to this file:
//   1) SCREW FLANGE  — module's 2 mounting-hole centre-to-centre spacing, and the
//                      offset from the TSOP centre to that hole line.
//   2) PCB SKIRT     — barrel drops to the PCB and grips around the TSOP legs for
//                      self-support: need the dome standoff height above the board.
//   3) BRACKET-BUILT — fold the barrel into your existing grey receiver mount
//                      (share that STL/measurements and I'll merge it).
