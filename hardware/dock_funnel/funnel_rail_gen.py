#!/usr/bin/env python3
# Copyright 2026 Logan Naidoo <naidoo.logan@gmail.com>
# SPDX-License-Identifier: Apache-2.0
#
# Parametric dock-funnel guide-rail generator for Jupiter.
#
# WHY: the original funnel wedged the robot because the flare widened too fast (~30 deg/side)
# and met the throat at a SHARP corner -> a skewed 280mm tongue drove its corner into the wall
# and locked (a heavy hand-push couldn't free it). This redesign:
#   * shallower flare (~24 deg) so a skewed tongue is guided IN, not caught,
#   * a ROUNDED flare->throat transition (fillet) -> kills the corner-catch (the main lock point),
#   * a slight throat taper with clearance -> accepts a few degrees of skew and squares it out.
# Pair it with: gentle software approach (keeps entry skew ~5 deg), chamfer the ROBOT's rear
# corners, and print in PETG (slicker + tougher than PLA).
#
# HARD CONSTRAINT: total funnel depth (FLARE_LEN + THROAT_LEN + FILLET_R) must stay BELOW the
# robot tongue length (~190mm) so the 385mm-wide DRIVE WHEELS never enter the funnel and collide
# with the flare. Keep total depth < ~175mm.
#
# Units: millimetres. Frame:  y = insertion depth (0 = mouth, +y into dock),  x = lateral
# (0 = centre axis; right rail x>0),  z = vertical (0 = base, +z up).
#
# Outputs (this folder): funnel_rail_right.stl, funnel_rail_left.stl, funnel_rail_profile.dxf
# Run:  python3 funnel_rail_gen.py

import math

# ===== PARAMETERS — tune these =============================================
MOUTH_W       = 385.0   # mouth opening width. <= robot drive-wheel width (385). DO NOT exceed.
THROAT_ENT_W  = 298.0   # throat entrance width. tongue 280 + skew allowance (298 accepts ~6 deg).
THROAT_SEAT_W = 285.0   # throat width at the seat. tongue 280 + ~5mm total clearance (taper squares it).
FLARE_LEN     = 105.0   # flare depth mouth->throat entrance. -> flare half-angle below.
THROAT_LEN    = 60.0    # throat depth (squaring runway).  total depth = FLARE_LEN+THROAT_LEN(+fillet)
FILLET_R      = 15.0    # radius blending flare into throat. THE key change (kills corner-catch).
WALL_H        = 50.0    # rail height.  >>> VERIFY <<< must catch the tongue AND clear the wheels.
WALL_T        = 5.0     # wall thickness (PETG). 5-6mm is sturdy.
FILLET_SEGS   = 10      # arc smoothness

TONGUE_W = 280.0        # reference only

# ===== geometry helpers ====================================================
def norm(v):
    m = math.hypot(v[0], v[1]);  return (v[0]/m, v[1]/m)

def fillet(A, B, C, R, segs):
    """Return arc points filleting the corner at B between segments B->A and B->C."""
    v1 = norm((A[0]-B[0], A[1]-B[1]))
    v2 = norm((C[0]-B[0], C[1]-B[1]))
    dot = max(-1.0, min(1.0, v1[0]*v2[0] + v1[1]*v2[1]))
    phi = math.acos(dot)                      # interior angle at B
    if phi < 1e-3 or abs(phi-math.pi) < 1e-3:
        return [B]
    d = R / math.tan(phi/2.0)                 # B -> tangent point along each leg
    T1 = (B[0]+v1[0]*d, B[1]+v1[1]*d)
    T2 = (B[0]+v2[0]*d, B[1]+v2[1]*d)
    bis = norm((v1[0]+v2[0], v1[1]+v2[1]))
    cen = (B[0]+bis[0]*(R/math.sin(phi/2.0)), B[1]+bis[1]*(R/math.sin(phi/2.0)))
    a1 = math.atan2(T1[1]-cen[1], T1[0]-cen[0])
    a2 = math.atan2(T2[1]-cen[1], T2[0]-cen[0])
    while a2 - a1 >  math.pi: a2 -= 2*math.pi
    while a2 - a1 < -math.pi: a2 += 2*math.pi
    return [(cen[0]+R*math.cos(a1+(a2-a1)*i/segs), cen[1]+R*math.sin(a1+(a2-a1)*i/segs))
            for i in range(segs+1)]

def inner_profile():
    """Guide-surface polyline (x>0 right rail), as (y, x) from mouth to seat."""
    P_mouth  = (0.0,               MOUTH_W/2.0)
    P_corner = (FLARE_LEN,         THROAT_ENT_W/2.0)          # flare meets throat (to be filleted)
    P_seat   = (FLARE_LEN+THROAT_LEN, THROAT_SEAT_W/2.0)
    # note: helper takes (x,y)-style tuples; here dim0=y(depth), dim1=x(lateral) — consistent throughout
    arc = fillet(P_mouth, P_corner, P_seat, FILLET_R, FILLET_SEGS)
    return [P_mouth] + arc + [P_seat]         # list of (y, x)

# ===== build a solid wall (extrude the footprint band in z) ================
def build_rail(mirror=False):
    inner = inner_profile()                                   # (y, x)
    s = -1.0 if mirror else 1.0
    inner = [(y, s*x) for (y, x) in inner]
    # outer edge: offset outward (away from centre) by WALL_T in x
    outer = [(y, x + s*WALL_T) for (y, x) in inner]
    n = len(inner)
    tris = []
    def V(pt, z): return (pt[1], pt[0], z)                    # -> (X_lateral, Y_depth, Z)
    def quad(a, b, c, d): tris.append((a,b,c)); tris.append((a,c,d))
    for i in range(n-1):
        ii, io, ji, jo = inner[i], outer[i], inner[i+1], outer[i+1]
        # bottom (z=0) and top (z=WALL_H) faces
        quad(V(ii,0), V(io,0), V(jo,0), V(ji,0))
        quad(V(ii,WALL_H), V(ji,WALL_H), V(jo,WALL_H), V(io,WALL_H))
        # inner side wall (the guide surface) and outer side wall
        quad(V(ii,0), V(ji,0), V(ji,WALL_H), V(ii,WALL_H))
        quad(V(io,0), V(io,WALL_H), V(jo,WALL_H), V(jo,0))
    # end caps
    quad(V(inner[0],0),  V(inner[0],WALL_H),  V(outer[0],WALL_H),  V(outer[0],0))
    quad(V(inner[-1],0), V(outer[-1],0),      V(outer[-1],WALL_H), V(inner[-1],WALL_H))
    return tris

def tri_normal(a, b, c):
    ux,uy,uz = b[0]-a[0], b[1]-a[1], b[2]-a[2]
    vx,vy,vz = c[0]-a[0], c[1]-a[1], c[2]-a[2]
    nx,ny,nz = uy*vz-uz*vy, uz*vx-ux*vz, ux*vy-uy*vx
    m = math.sqrt(nx*nx+ny*ny+nz*nz) or 1.0
    return (nx/m, ny/m, nz/m)

def write_stl(path, tris, name):
    with open(path, "w") as f:
        f.write(f"solid {name}\n")
        for a,b,c in tris:
            nx,ny,nz = tri_normal(a,b,c)
            f.write(f"  facet normal {nx:.6e} {ny:.6e} {nz:.6e}\n    outer loop\n")
            for v in (a,b,c):
                f.write(f"      vertex {v[0]:.4f} {v[1]:.4f} {v[2]:.4f}\n")
            f.write("    endloop\n  endfacet\n")
        f.write(f"endsolid {name}\n")

def write_dxf(path):
    """2D footprint of the RIGHT rail (inner + outer edge, closed) for Fusion sketch->extrude."""
    inner = inner_profile()
    outer = [(y, x + WALL_T) for (y, x) in inner]
    loop = [(x, y) for (y, x) in inner] + [(x, y) for (y, x) in reversed(outer)]
    with open(path, "w") as f:
        f.write("0\nSECTION\n2\nENTITIES\n0\nLWPOLYLINE\n8\n0\n90\n%d\n70\n1\n" % len(loop))
        for (x, y) in loop:
            f.write("10\n%.4f\n20\n%.4f\n" % (x, y))
        f.write("0\nENDSEC\n0\nEOF\n")

if __name__ == "__main__":
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    right = build_rail(mirror=False)
    left  = build_rail(mirror=True)
    write_stl(os.path.join(here, "funnel_rail_right.stl"), right, "funnel_rail_right")
    write_stl(os.path.join(here, "funnel_rail_left.stl"),  left,  "funnel_rail_left")
    write_dxf(os.path.join(here, "funnel_rail_profile.dxf"))
    flare_ang = math.degrees(math.atan2((MOUTH_W-THROAT_ENT_W)/2.0, FLARE_LEN))
    total_depth = FLARE_LEN + THROAT_LEN
    print("=== funnel rail generated ===")
    print(f"mouth {MOUTH_W}  throat-entrance {THROAT_ENT_W}  throat-seat {THROAT_SEAT_W}  (tongue {TONGUE_W})")
    print(f"flare half-angle : {flare_ang:.1f} deg   (was ~30 deg; target 15-25)")
    print(f"fillet radius    : {FILLET_R} mm  (was a sharp corner)")
    print(f"total funnel depth: {total_depth:.0f} mm  (MUST stay < ~175 so drive wheels clear)")
    print(f"wall height {WALL_H}  thickness {WALL_T}   triangles/rail: {len(right)}")
    # skew acceptance at the throat entrance:
    import numpy as np
    for th in (2,3,4,5,6):
        eff = TONGUE_W*math.cos(math.radians(th)) + 190*math.sin(math.radians(th))
        ok = "OK" if eff <= THROAT_ENT_W else "JAM"
        print(f"  tongue at {th} deg skew -> {eff:5.1f} mm  vs throat-ent {THROAT_ENT_W}  [{ok}]")
