"""
Pi Box - full Nanuk 909 build: screen in the lid, keyboard in the base, Pi + Anker 737
battery under the keyboard, cable trough along the hinge side.

    python pibox_case.py            -> writes parts/*.stl, parts/*.3mf, assembly + fit report

World frame (mm), case closed and sitting upright:
  X = long axis (0 = left edge of the bottom bezel plate), Y = short axis (0 = front/latch side),
  Z = up, 0 = deepest point of the base floor.
Every printable part is exported in its own print orientation (flat face on the bed) and fits
a 256 x 256 bed. The Pi cage is the v2 undercarriage geometry (undercarriage_v2.py).

Assumed values are marked ASSUMED - measure them and re-run to tighten the fit.
"""
import math
import os
import numpy as np
import manifold3d as m3
import trimesh

import undercarriage_v2 as v2
from undercarriage_v2 import box, prism, xz_prism, hull2, to_trimesh, write_3mf

IN = 25.4
HERE = os.path.dirname(os.path.abspath(__file__))

# ============================================================== components (from listings)
KB_L, KB_W, KB_T = 279.4, 132.1, 20.3        # SR mini keyboard 11 x 5.2 x 0.8 in
KB_CABLE_X = 116.0                           # ASSUMED: cable exits the back edge near F5
BAT_L, BAT_W, BAT_H = 155.7, 54.6, 49.5      # Anker 737 (length along Y, lies flat)
SCR_W, SCR_H, SCR_T = 9.25 * IN, 5.62 * IN, 0.27 * IN   # HMTECH 10.1" (measured by user)
SCR_PORT_SIDE = 'left'                       # ASSUMED: HDMI/USB on the left short edge

# ============================================================== Nanuk 909 (Plasticase drawings)
BZ_W, BZ_H, BZ_R = 11.995 * IN, 7.495 * IN, 0.692 * IN   # bottom bezel plate
XC, YC = BZ_W / 2, BZ_H / 2                               # case centre in world XY
LZ_W, LZ_H, LZ_R = 11.786 * IN, 7.286 * IN, 0.595 * IN   # lid bezel plate (concentric)
PLATE_T = 0.125 * IN
Z_PB = 2.240 * IN              # base: floor (deepest) -> underside of bezel plate
Z_PT = Z_PB + PLATE_T          # base bezel top ~= base rim
LID_GAP = 1.9                  # ASSUMED: base plate top -> lid plate face when closed
Z_LP = Z_PT + LID_GAP          # lid plate face (the side facing the base)
Z_LB = Z_LP + PLATE_T          # lid boss faces
Z_CEIL = Z_LB + 0.811 * IN     # lid inside ceiling (drawing: .811 from plate)
BOSS_R = 5.0                   # ASSUMED radius of the moulded screw bosses
BOSS_DEPTH_BASE = 0.400 * IN

BASE_DXF_ORIGIN = (5.93668, 10.72774)
BASE_HOLES_DXF = [(17.594, 11.064), (6.274, 11.064), (17.648, 13.344), (8.758, 10.886),
                  (17.648, 15.607), (10.805, 11.011), (17.594, 17.886), (13.063, 11.011),
                  (15.110, 10.886), (15.110, 18.064), (13.063, 17.939), (10.805, 17.939),
                  (8.758, 18.064), (6.274, 17.886), (6.220, 15.607), (6.220, 13.344)]
LID_HOLES_DXF = [(18.289, 16.642), (8.359, 11.321), (14.093, 16.642), (9.898, 9.680),
                 (9.898, 16.642), (14.093, 9.680), (8.359, 15.008), (18.289, 9.680),
                 (19.827, 11.321), (19.827, 15.008)]
LID_DXF_CENTRE = (14.093, 13.161)

BASE_HOLES = [((x - BASE_DXF_ORIGIN[0]) * IN, (y - BASE_DXF_ORIGIN[1]) * IN) for x, y in BASE_HOLES_DXF]
LID_HOLES = [(XC + (x - LID_DXF_CENTRE[0]) * IN, YC + (y - LID_DXF_CENTRE[1]) * IN) for x, y in LID_HOLES_DXF]

HOLE_D = 0.170 * IN + 0.1
CSK_D = 0.296 * IN + 0.2
PILOT_D = 2.5                  # M3 self-tapping into PLA
M3_CLR = 3.4

# ============================================================== layout
KB_CLR = 0.8
KX0, KX1 = XC - KB_L / 2 - KB_CLR, XC + KB_L / 2 + KB_CLR     # keyboard pocket
KY0 = 12.7
KY1 = KY0 + KB_W + 2 * KB_CLR
DIV_T = 2.0
TY0, TY1 = KY1 + DIV_T, 176.5                                 # cable trough (hinge side)
TX0, TX1 = 14.0, BZ_W - 14.0
Z_KF = BAT_H + 0.5             # keyboard floor (top of the module plates)
MOD_T = PLATE_T                # module plate thickness (= v2 plate, so the cage drops in as-is)
BX0 = 185.0                    # battery footprint (world)
BX1 = BX0 + BAT_W
BY0 = 10.3
BY1 = BY0 + BAT_L
SEAM = XC

# ============================================================== helpers
def rrect(x0, x1, y0, y1, r, n=12):
    pts = []
    for cx, cy, a0 in [(x1 - r, y0 + r, -90), (x1 - r, y1 - r, 0), (x0 + r, y1 - r, 90), (x0 + r, y0 + r, 180)]:
        for i in range(n + 1):
            a = math.radians(a0 + 90 * i / n)
            pts.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    return pts


def cyl(x, y, z0, z1, r, n=48):
    return m3.Manifold.cylinder(z1 - z0, r, r, n).translate([x, y, z0])


def countersink(x, y, z_face, down=True):
    """#6 flat-head hole, 82 deg countersink opening on z_face (down=True: hole goes -Z)."""
    depth = (CSK_D - HOLE_D) / 2 / math.tan(math.radians(41))
    if down:
        cone = m3.Manifold.cylinder(depth + 0.01, HOLE_D / 2, CSK_D / 2, 64).translate([x, y, z_face - depth])
        thru = cyl(x, y, z_face - 30, z_face + 1, HOLE_D / 2, 64)
        lip = cyl(x, y, z_face, z_face + 1, CSK_D / 2, 64)
    else:
        cone = m3.Manifold.cylinder(depth + 0.01, CSK_D / 2, HOLE_D / 2, 64).translate([x, y, z_face])
        thru = cyl(x, y, z_face - 1, z_face + 30, HOLE_D / 2, 64)
        lip = cyl(x, y, z_face - 1, z_face, CSK_D / 2, 64)
    return cone + thru + lip


def mx(man):
    """Mirror about the case centre line (left half -> right half)."""
    return man.mirror([1, 0, 0]).translate([BZ_W, 0, 0])


def half(man, side):
    big = 1000
    if side == 'L':
        return man ^ box(-big, SEAM, -big, big, -big, big)
    return man ^ box(SEAM, big, -big, big, -big, big)


def flip_z(man, z_top):
    """Print orientation: face at world z_top goes onto the bed (z=0), part grows +Z."""
    return man.mirror([0, 0, 1]).translate([0, 0, z_top])


def to_bed(man):
    b = man.bounding_box()
    return man.translate([-b[0], -b[1], -b[2]])


# ============================================================== base frame (replaces bezel plate)
FRONT_POSTS = {'L': [(40.0, 9.2), (97.0, 9.2)], 'R': [(160.0, 9.2), (265.0, 9.2)]}
END_POSTS = {'L': [(8.85, 30.0), (8.85, 95.0), (8.85, 160.0)],
             'R': [(BZ_W - 8.85, 30.0), (BZ_W - 8.85, 95.0), (BZ_W - 8.85, 160.0)]}
DIV_POSTS = {'L': [(40.0, TY0 + 2.0), (97.0, TY0 + 2.0)], 'R': [(170.0, TY0 + 2.0), (265.0, TY0 + 2.0)]}
BACK_POSTS = {'L': [(100.0, TY1 - 2.0), (130.0, TY1 - 2.0)], 'R': [(174.67, TY1 - 2.0), (265.0, TY1 - 2.0)]}
USBC = dict(x=270.0, y=(TY0 + TY1) / 2, slot=(12.4, 6.4), hole_dx=12.5)   # ASSUMED panel socket


def all_posts(side):
    return FRONT_POSTS[side] + END_POSTS[side] + DIV_POSTS[side] + BACK_POSTS[side]


def base_frame():
    outline = []
    r, w, h = BZ_R, BZ_W, BZ_H
    for cx, cy, a0 in [(w - r, r, -90), (w - r, h - r, 0), (r, h - r, 90), (r, r, 180)]:
        for i in range(25):
            a = math.radians(a0 + 90 * i / 24)
            outline.append((cx + r * math.cos(a), cy + r * math.sin(a)))
    f = prism(outline, Z_PB, Z_PT)
    f -= box(KX0, KX1, KY0, KY1, Z_PB - 1, Z_PT + 1)              # keyboard opening
    f -= box(TX0, TX1, TY0, TY1, Z_PB - 1, Z_PT + 1)              # cable trough opening
    # divider between keyboard and trough (also the keyboard's back stop), full height
    f += box(KX0, KX1, KY1, TY0, Z_KF, Z_PT)
    f -= box(KB_CABLE_X - 12, KB_CABLE_X + 12, KY1 - 1, TY0 + 1, Z_KF + 2.0, Z_PT + 1)
    # trough back wall
    f += box(TX0, TX1, TY1 - 2.0, TY1, Z_KF, Z_PB + 0.01)
    # USB-C charge socket platform (flush with the deck top)
    u = USBC
    f += box(u['x'] - 14, u['x'] + 14, TY0 - 0.01, TY1 + 0.01, Z_PT - 2.2, Z_PT)
    f -= box(u['x'] - u['slot'][0] / 2, u['x'] + u['slot'][0] / 2, u['y'] - u['slot'][1] / 2,
             u['y'] + u['slot'][1] / 2, Z_PT - 5, Z_PT + 1)
    for dx in (-u['hole_dx'], u['hole_dx']):
        f -= cyl(u['x'] + dx, u['y'], Z_PT - 5, Z_PT + 1, 1.6, 32)
    # mounting posts for the modules (pilot holes from below)
    for side in 'LR':
        for (x, y) in all_posts(side):
            if (x, y) in DIV_POSTS[side]:
                p = box(x - 4, x + 4, TY0 - 0.01, TY0 + 4.5, Z_KF, Z_PB + 0.01)
            elif (x, y) in BACK_POSTS[side]:
                p = box(x - 4, x + 4, TY1 - 4.5, TY1, Z_KF, Z_PB + 0.01)
            elif (x, y) in FRONT_POSTS[side]:
                p = box(x - 3.5, x + 3.5, 5.7, KY0, Z_KF, Z_PB + 0.01)
            else:
                xa, xb = (5.85, KX0) if side == 'L' else (KX1, BZ_W - 5.85)
                p = box(xa, xb, y - 3.5, y + 3.5, Z_KF, Z_PB + 0.01)
            f += p
            f -= cyl(x, y, Z_KF - 1, Z_KF + 9, PILOT_D / 2, 24)
    # stay clear of the case's own screw bosses under the plate
    for (x, y) in BASE_HOLES:
        f -= cyl(x, y, Z_PB - BOSS_DEPTH_BASE - 1, Z_PB - 0.01, BOSS_R + 0.5)
    for (x, y) in BASE_HOLES:
        f -= countersink(x, y, Z_PT, down=True)
    return f


# ============================================================== modules (keyboard floor + under-deck)
def module_plate(side):
    if side == 'L':
        pts = rrect(5.85, SEAM - 0.2, 5.7, TY1, 10)
    else:
        pts = rrect(SEAM + 0.2, BZ_W - 5.85, 5.7, TY1, 10)
    p = prism(pts, Z_KF - MOD_T, Z_KF)
    for (x, y) in BASE_HOLES:
        p -= cyl(x, y, Z_KF - 10, Z_KF + 1, BOSS_R + 0.6)
    for (x, y) in all_posts(side):
        p -= cyl(x, y, Z_KF - 10, Z_KF + 1, M3_CLR / 2, 24)
    return p


def pi_module():
    """Left half: keyboard floor + DeskPi cage (v2 undercarriage geometry, hung under it)."""
    half_cage = v2.side_frame() + v2.latch_arm()
    cage = v2.end_wall() + v2.rails() + half_cage + v2.mirror_y(half_cage)
    cage = cage.mirror([0, 0, 1]).translate([0, 0, Z_KF])      # v2 local z (down) -> world
    m = module_plate('L') + cage
    m -= box(30, 84, 169.5, 175.5, Z_KF - 10, Z_KF + 1)          # Pi rear-port cable drop
    m -= box(118, 140, TY0 + 6, TY1 - 3, Z_KF - 10, Z_KF + 1)     # keyboard cable drop
    return m


def battery_module():
    """Right half: keyboard floor with a window over the battery + a sleeve that holds it."""
    m = module_plate('R')
    g = 0.4
    m -= box(BX0 - g, BX1 + g, BY0 - g, BY1 + g, Z_KF - 10, Z_KF + 1)   # keyboard rests over battery
    zb = 15.0
    for xa, xb in [(BX0 - g - 2.4, BX0 - g), (BX1 + g, BX1 + g + 2.4)]:
        m += box(xa, xb, BY0 + 3, BY1 - 12, zb, Z_KF - MOD_T + 0.01)
        # vertical friction ribs (half-round, 0.6 mm)
        face = xb if xa < BX0 else xa
        sgn = 1 if xa < BX0 else -1
        for y in np.linspace(BY0 + 15, BY1 - 25, 5):
            m += m3.Manifold.cylinder(Z_KF - MOD_T - zb, 1.2, 1.2, 24).translate([face - sgn * 0.6, y, zb])
    m += box(BX0 - g - 2.4, BX1 + g + 2.4, BY0 - g - 2.0, BY0 - g, 28.0, Z_KF - MOD_T + 0.01)  # front stop
    for xa in (BX0 - g - 2.4, BX1 + g - 6.0):                                   # rear corner stops
        m += box(xa, xa + 8.4, BY1 + g, BY1 + g + 2.0, 28.0, Z_KF - MOD_T + 0.01)
    m -= box(BX0 + 2, BX1 - 2, BY1 + 1.5, TY1 - 1.0, Z_KF - 10, Z_KF + 1)     # battery port cables up
    m -= box(156, 180, TY0 + 4, TY1 - 3, Z_KF - 10, Z_KF + 1)                 # cables down to Pi
    return m


# ============================================================== lid screen frame
SX0, SX1 = XC - SCR_W / 2, XC + SCR_W / 2
SY0, SY1 = YC - SCR_H / 2, YC + SCR_H / 2
SCR_CLR, TRAY_WALL, BACK_T, LIP_T, LIP_W = 0.4, 2.0, 1.6, 1.2, 3.5
TAPE = 0.5
Z_BACK_TOP = Z_CEIL - 1.0
Z_SCR_BACK = Z_BACK_TOP - BACK_T - TAPE
Z_SCR_FRONT = Z_SCR_BACK - SCR_T
Z_LIP = Z_SCR_FRONT - LIP_T
ARM_Z0, ARM_Z1 = Z_LIP, Z_LIP + 4.0


def lid_frame():
    ix0, ix1 = SX0 - SCR_CLR, SX1 + SCR_CLR
    iy0, iy1 = SY0 - SCR_CLR, SY1 + SCR_CLR
    ox0, ox1, oy0, oy1 = ix0 - TRAY_WALL, ix1 + TRAY_WALL, iy0 - TRAY_WALL, iy1 + TRAY_WALL
    t = box(ox0, ox1, oy0, oy1, Z_LIP, Z_BACK_TOP)
    t -= box(ix0, ix1, iy0, iy1, Z_SCR_FRONT, Z_SCR_BACK + TAPE + 0.01)            # screen pocket
    t -= box(ix0 + LIP_W, ix1 - LIP_W, iy0 + LIP_W, iy1 - LIP_W, Z_LIP - 1, Z_SCR_FRONT + 0.01)  # window
    # vents in the back plate (diamonds, print fine either way)
    for gx in np.linspace(ix0 + 30, ix1 - 30, 6):
        for gy in np.linspace(iy0 + 30, iy1 - 30, 3):
            t -= m3.Manifold.cube([16, 16, 10], center=True).rotate([0, 0, 45]).translate([gx, gy, Z_BACK_TOP])
    # port opening on the port side (wall + lip), back plate kept
    px = (ox0 - 1, ix0 + 0.01) if SCR_PORT_SIDE == 'left' else (ix1 - 0.01, ox1 + 1)
    t -= box(px[0], px[1], iy0 + 38, iy1 - 38, Z_LIP - 1, Z_SCR_BACK)
    # cable channel along the port end, toward the hinge side
    cx0, cx1 = 14.0, ox0
    if SCR_PORT_SIDE == 'left':
        ch = box(cx0, cx1 + 0.01, iy0 + 20, oy1, ARM_Z0, ARM_Z0 + 1.6)              # floor
        ch += box(cx0, cx0 + 1.6, iy0 + 20, oy1, ARM_Z0, Z_BACK_TOP - 3)            # outer wall
        for y in np.linspace(iy0 + 30, oy1 - 10, 4):                                 # retaining fingers
            ch += box(cx0, cx0 + 6, y, y + 4, Z_BACK_TOP - 4.6, Z_BACK_TOP - 3)
        t += ch
    # tabs on the case's lid bosses: riser beside each boss + arm into the tray
    parts = t
    for (hx, hy) in LID_HOLES:
        if abs(hx - XC) < 1:          # the two centre holes sit on the split line - not used
            continue
        if abs(hy - YC) > 60:         # front / back rows: riser beside the boss along X
            sx = 1 if hx < XC else -1
            rx0, rx1 = sorted([hx + sx * (BOSS_R + 0.5), hx + sx * (BOSS_R + 6.5)])
            if hy < YC:
                ry0, ry1 = 6.0, KY0 - 0.3
                arm = box(rx0, rx1, ry0, oy0 + 1, ARM_Z0, ARM_Z1)
            else:
                ry0, ry1 = BZ_H - 13.5, BZ_H - 6.0
                arm = box(rx0, rx1, oy1 - 1, ry1, ARM_Z0, ARM_Z1)
        else:                         # side holes: riser beside the boss along Y
            sy = 1 if hy < YC else -1
            ry0, ry1 = sorted([hy + sy * (BOSS_R + 0.5), hy + sy * (BOSS_R + 6.5)])
            if hx < XC:
                rx0, rx1 = 6.0, KX0 - 0.6
                arm = box(rx0, ox0 + 1, ry0, ry1, ARM_Z0, ARM_Z1)
            else:
                rx0, rx1 = KX1 + 0.6, BZ_W - 6.0
                arm = box(ox1 - 1, rx1, ry0, ry1, ARM_Z0, ARM_Z1)
        riser = box(rx0, rx1, ry0, ry1, Z_LP, ARM_Z1)
        tab = hull2(cyl(hx, hy, Z_LP, Z_LB, 5.0), box(rx0, rx1, ry0, ry1, Z_LP, Z_LB))
        parts += riser + arm + tab
    for (hx, hy) in LID_HOLES:
        parts -= cyl(hx, hy, Z_LB, Z_CEIL + 5, BOSS_R + 0.4)        # clear the bosses above the tabs
        parts -= countersink(hx, hy, Z_LP, down=False)
    # seam joints: bolt blocks on the front and back walls (M3 x 16 through both halves)
    for (ya, yb) in [(oy0 - 7, oy0 + 0.01), (oy1 - 0.01, oy1 + 7)]:
        blk = box(SEAM - 6, SEAM + 6, ya, yb, Z_LIP + 2, Z_BACK_TOP)
        parts += blk
        parts -= m3.Manifold.cylinder(30, M3_CLR / 2, M3_CLR / 2, 24).rotate([0, 90, 0]) \
            .translate([SEAM - 15, (ya + yb) / 2, (Z_LIP + 2 + Z_BACK_TOP) / 2])
        for sx in (-1, 1):   # nut trap / head pocket
            parts -= m3.Manifold.cylinder(10, 3.3, 3.3, 6).rotate([0, 90, 0]) \
                .translate([SEAM + sx * 4.0 if sx > 0 else SEAM - 14.0, (ya + yb) / 2, (Z_LIP + 2 + Z_BACK_TOP) / 2])
    return parts


# ============================================================== stand-ins for checking
def keyboard():
    return box(XC - KB_L / 2, XC + KB_L / 2, KY0 + KB_CLR, KY0 + KB_CLR + KB_W, Z_KF, Z_KF + KB_T)


def battery():
    return box(BX0, BX1, BY0, BY1, 0.0, BAT_H)


def screen():
    return box(SX0, SX1, SY0, SY1, Z_SCR_FRONT, Z_SCR_BACK)


def pibox():
    x1 = v2.PX1
    return box(x1 - v2.BOX_W, x1, v2.YC - v2.BOX_L / 2, v2.YC + v2.BOX_L / 2,
               Z_KF - v2.ZB, Z_KF - v2.ZB + v2.BOX_H)


def bosses():
    b = m3.Manifold()
    for (x, y) in BASE_HOLES:
        b += cyl(x, y, Z_PB - BOSS_DEPTH_BASE, Z_PB, BOSS_R)
    for (x, y) in LID_HOLES:
        b += cyl(x, y, Z_LB, Z_LB + 0.298 * IN + 3, BOSS_R)
    return b


def interior_floor():
    """Rough check volume: case interior at the floor is 11.44 x 7.0 in."""
    w, h = 11.44 * IN, 7.0 * IN
    return box(XC - w / 2, XC + w / 2, YC - h / 2, YC + h / 2, 0, Z_CEIL)


# ============================================================== build / export
def clear_bosses(man):
    for (x, y) in BASE_HOLES:
        man -= cyl(x, y, Z_PB - BOSS_DEPTH_BASE - 2, Z_PB, BOSS_R + 0.6)
    return man


def build_all():
    bf = base_frame()
    lf = lid_frame()
    return {
        'base_frame_L': half(bf, 'L'),
        'base_frame_R': half(bf, 'R'),
        'pi_module': clear_bosses(pi_module()),
        'battery_module': clear_bosses(battery_module()),
        'lid_frame_L': half(lf, 'L'),
        'lid_frame_R': half(lf, 'R'),
    }


PRINT_TOP = {   # world z of the face that goes on the bed (flipped) / None = already bed-down
    'base_frame_L': Z_PT, 'base_frame_R': Z_PT,
    'pi_module': Z_KF, 'battery_module': Z_KF,
    'lid_frame_L': None, 'lid_frame_R': None,
}


def fit_report(parts):
    lines = []
    def gap(name, a, b):
        v = (a ^ b).volume()
        lines.append('  %-38s %s' % (name, 'OK (no overlap)' if v < 0.5 else 'OVERLAP %.1f mm3' % v))
    kb, bat, scr, pb = keyboard(), battery(), screen(), pibox()
    for n, p in parts.items():
        gap('%s vs keyboard' % n, p, kb)
    for n, p in parts.items():
        gap('%s vs battery' % n, p, bat)
    for n, p in parts.items():
        gap('%s vs case bosses' % n, p, bosses())
    gap('pi box vs battery', pb, bat)
    gap('keyboard vs case bosses', kb, bosses())
    lines.append('')
    lines.append('  stack (world Z, floor = 0):')
    lines.append('    battery top            %6.1f' % BAT_H)
    lines.append('    keyboard floor         %6.1f' % Z_KF)
    lines.append('    keyboard top           %6.1f' % (Z_KF + KB_T))
    lines.append('    lid frame lowest part  %6.1f   (lip / arms over the keyboard)' % Z_LIP)
    lines.append('    -> keyboard clearance  %6.1f mm' % (Z_LIP - Z_KF - KB_T))
    lines.append('    screen front / back    %6.1f / %.1f' % (Z_SCR_FRONT, Z_SCR_BACK))
    lines.append('    lid ceiling            %6.1f   (back plate top %.1f)' % (Z_CEIL, Z_BACK_TOP))
    lines.append('    Pi cage bottom         %6.1f' % (Z_KF - v2.ZL))
    return '\n'.join(lines)


if __name__ == '__main__':
    out = os.path.join(HERE, 'parts')
    os.makedirs(out, exist_ok=True)
    parts = build_all()
    world = {}
    for name, man in parts.items():
        assert man.status() == m3.Error.NoError, name
        world[name] = man
        pm = man if PRINT_TOP[name] is None else flip_z(man, PRINT_TOP[name])
        if name.startswith('lid_frame'):
            pm = flip_z(man, Z_BACK_TOP)      # ceiling side on the bed
        pm = to_bed(pm)
        tm = to_trimesh(pm)
        bb = pm.bounding_box()
        print('%-16s %6.1f x %6.1f x %5.1f mm  watertight=%s  %.0f cm3' %
              (name, bb[3] - bb[0], bb[4] - bb[1], bb[5] - bb[2], tm.is_watertight, tm.volume / 1000))
        tm.export(os.path.join(out, name + '.stl'))
        write_3mf(tm, os.path.join(out, name + '.3mf'))
    # assembly in world coordinates (parts + stand-ins), for previews
    asm = {k: to_trimesh(v) for k, v in world.items()}
    for k, f in [('keyboard', keyboard), ('battery', battery), ('screen', screen), ('pibox', pibox)]:
        asm['_' + k] = to_trimesh(f())
    np.save(os.path.join(out, '_assembly.npy'), {k: (v.vertices, v.faces) for k, v in asm.items()}, allow_pickle=True)
    rep = fit_report(world)
    print(rep)
    with open(os.path.join(out, 'fit_report.txt'), 'w') as fh:
        fh.write(rep + '\n')
