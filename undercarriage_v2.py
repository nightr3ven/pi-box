"""
Pi undercarriage v2 - Nanuk 909 (bottom bezel, left half) + DeskPi Lite Pi5 cradle.

Parametric: edit the BOX_* numbers below (measure your case with calipers) and run:
    python undercarriage_v2.py
Outputs undercarriage_v2.stl and undercarriage_v2.3mf next to this script.

Print orientation = as modelled: plate flat on the bed (countersink side DOWN on the bed),
cage growing up. Installed in the case it is flipped: plate on top, cage hanging down.

Frame (mm): X 0 = left edge of the bezel plate, Y 0 = front edge of the plate,
Z 0 = TOP face of the plate when installed (the countersunk face).
The box slides in along -X from the open +X end and clicks behind two side latch arms.
"""
import math
import numpy as np
import manifold3d as m3
import trimesh

IN = 25.4

# ---------------------------------------------------------------- box (DeskPi Lite Pi5)
# Published (approx.) size is 147 x 94 x 34 mm. Your old pocket was 146.8 x 93.2 x 35.2,
# so your real case is probably a bit smaller -- the spring fingers take up the slack.
BOX_L = 147.0   # along Y (long side, port ends face +/-Y)
BOX_W = 94.0    # along X (slide-in direction)
BOX_H = 34.0    # along Z
CLEAR = 0.3     # rigid clearance per side (X and Y) on top of the box size
CLEAR_Z = 0.2   # per side in Z (box is clamped between standoff rails and ledges)

# ---------------------------------------------------------------- Nanuk 909 bezel (BTM)
# From 909_LEXAN BEZEL PLATE_BTM_p.01 (Plasticase drawing, inches).
PLATE_T = 0.125 * IN                     # 3.175, recommended bezel thickness
PLATE_W = 11.995 / 2 * IN                # left half, cut on the centreline -> 152.34
PLATE_H = 7.495 * IN                     # 190.37
CORNER_R = 0.692 * IN                    # 17.58
DXF_ORIGIN = (5.93668, 10.72774)         # plate lower-left corner in the DXF (inches)
HOLES_DXF = [(6.274, 11.064), (6.274, 17.886), (6.220, 13.344), (6.220, 15.607),
             (8.758, 10.886), (8.758, 18.064), (10.805, 11.011), (10.805, 17.939)]
HOLE_D = 0.170 * IN + 0.1                # 4.42  (#6 clearance, +0.1 for printing)
CSK_D = 0.296 * IN + 0.2                 # 7.72  (82 deg, heads sit flush/just below)
CSK_ANGLE = 82.0

# ---------------------------------------------------------------- cage layout
AIR_GAP = 3.0            # standoff rails: air gap between plate and top of the box
LEDGE_T = 3.0            # thickness of the bottom ledges the box rests on
CAGE_X0 = 11.43          # outer face of the closed (X-) wall - same as v1 (known fit)
WALL_T = 3.57
POST_Y0 = 14.2           # outer face of the side frames - same envelope as v1
FINGER_T = 1.6           # spring finger thickness (tip); root is thicker
X_FINGER_REACH = 2.6     # how far the X- wall fingers stick into the pocket
Y_FINGER_REACH = 1.5     # side fingers at the closed end
ARM_BUMP_REACH = 1.6     # side-press bump on the latch arms
HOOK_REACH = 2.2         # how far the latch hook reaches past the box side face
ARM_T = 1.8
ARM_ROOT_X = 141.5       # latch arm length ~32 mm -> ~0.6% strain in PLA when opened
SLOPE = 0.7              # dz/dx of arm underside (55 deg from vertical, self-supporting)

# derived pocket
PX0 = CAGE_X0 + WALL_T                       # 15.0  closed-end face
PX1 = PX0 + BOX_W + 2 * CLEAR                # hook catch face
YC = PLATE_H / 2
PY0 = YC - (BOX_L + 2 * CLEAR) / 2
PY1 = YC + (BOX_L + 2 * CLEAR) / 2
ZT = PLATE_T + AIR_GAP                       # top of box (installed) touches here
ZB = ZT + BOX_H + 2 * CLEAR_Z                 # bottom of box rests on ledges here
ZL = ZB + LEDGE_T                            # cage depth below plate top
POST_Y1 = PY0                                # inner face of side frames
XPOST = (PX1 - 23.6, PX1 - 13.6)             # +X corner posts (anchor for side bars)


# ---------------------------------------------------------------- helpers
def box(x0, x1, y0, y1, z0, z1):
    return m3.Manifold.cube([x1 - x0, y1 - y0, z1 - z0]).translate([x0, y0, z0])


def prism(poly_xy, z0, z1):
    cs = m3.CrossSection([poly_xy], m3.FillRule.NonZero)
    if cs.area() < 0:
        cs = m3.CrossSection([poly_xy[::-1]], m3.FillRule.NonZero)
    return cs.extrude(z1 - z0).translate([0, 0, z0])


def xz_prism(poly_xz, y0, y1):
    """Polygon in the XZ plane extruded along +Y from y0 to y1."""
    cs = m3.CrossSection([poly_xz], m3.FillRule.NonZero)
    if cs.area() < 0:
        cs = m3.CrossSection([poly_xz[::-1]], m3.FillRule.NonZero)
    # extrude along Z then rotate so that local (x, y, z) -> (x, z, y)
    s = cs.extrude(y1 - y0)
    s = s.rotate([90, 0, 0])            # (x, y, z) -> (x, -z, y)
    return s.translate([0, y1, 0])          # rotate maps extrusion e -> y = y1 - e


def arc_bump(cx, cy, half_w, reach, direction, n=24):
    """Circular-segment profile (XY) bulging `reach` toward `direction` ('+x','+y','-y')."""
    r = (half_w ** 2 + reach ** 2) / (2 * reach)
    pts = []
    a = math.asin(half_w / r)
    for i in range(n + 1):
        t = -a + 2 * a * i / n
        u = r * math.sin(t)                  # along the face
        v = r * math.cos(t) - (r - reach)    # out of the face
        pts.append((u, v))
    pts = [(-half_w, -0.6)] + pts + [(half_w, -0.6)]
    if direction == '+x':
        return [(cx + v, cy + u) for u, v in pts]
    if direction == '+y':
        return [(cx + u, cy + v) for u, v in pts]
    if direction == '-y':
        return [(cx + u, cy - v) for u, v in pts]
    raise ValueError


def hull2(a, b):
    return (a + b).hull()


def mirror_y(man):
    return man.mirror([0, 1, 0]).translate([0, PLATE_H, 0])


# ---------------------------------------------------------------- plate
def plate():
    r, w, h = CORNER_R, PLATE_W, PLATE_H
    pts = []
    for i in range(33):                       # lower-left corner arc
        a = math.pi + (math.pi / 2) * i / 32
        pts.append((r + r * math.cos(a), r + r * math.sin(a)))
    pts += [(w, 0), (w, h)]
    for i in range(33):                       # upper-left corner arc
        a = math.pi / 2 + (math.pi / 2) * i / 32
        pts.append((r + r * math.cos(a), h - r + r * math.sin(a)))
    p = prism(pts, 0, PLATE_T)

    csk_depth = (CSK_D - HOLE_D) / 2 / math.tan(math.radians(CSK_ANGLE / 2))
    for hx, hy in HOLES_DXF:
        x = (hx - DXF_ORIGIN[0]) * IN
        y = (hy - DXF_ORIGIN[1]) * IN
        thru = m3.Manifold.cylinder(PLATE_T + 2, HOLE_D / 2, HOLE_D / 2, 64).translate([x, y, -1])
        cone = m3.Manifold.cylinder(csk_depth + 0.01, CSK_D / 2, HOLE_D / 2, 64).translate([x, y, -0.005])
        lip = m3.Manifold.cylinder(0.5, CSK_D / 2, CSK_D / 2, 64).translate([x, y, -0.5])
        p = p - thru - cone - lip
    return p


# ---------------------------------------------------------------- closed end wall (X-)
FINGER_YS = [YC - 40.0, YC + 40.0]
DIAMOND_YS = [YC - 65.0, YC - 20.0, YC, YC + 20.0, YC + 65.0]


def finger_x(yc, w=8.0):
    """Vertical spring finger in the X- wall, root at the plate, bump pushes the box +X."""
    ztop = ZB - 3.5
    root_t = FINGER_T + 0.8
    # tapered blade: thicker at the root, FINGER_T at the tip, inner face flush with PX0
    blade = xz_prism([(PX0 - root_t, PLATE_T - 0.01), (PX0, PLATE_T - 0.01),
                      (PX0, ztop), (PX0 - FINGER_T, ztop)], yc - w / 2, yc + w / 2)
    bump = prism(arc_bump(PX0, yc, w / 2, X_FINGER_REACH, '+x'), ztop - 11, ztop)
    # chamfer the bump's lower end so it prints without overhang
    lead = hull2(prism(arc_bump(PX0, yc, w / 2, X_FINGER_REACH, '+x'), ztop - 11, ztop - 10.99),
                            box(PX0 - 0.6, PX0, yc - w / 2, yc + w / 2, ztop - 11 - X_FINGER_REACH - 1, ztop - 11))
    return blade + bump + lead


def end_wall():
    w = box(CAGE_X0, PX0, POST_Y0, PLATE_H - POST_Y0, 0.5, ZL)
    # finger windows (through the wall), leave 1.5 mm of wall above for the ledge
    for yc in FINGER_YS:
        w -= box(CAGE_X0 - 1, PX0 + 1, yc - 5.5, yc + 5.5, PLATE_T, ZB - 2.0)
    # diamond vents (45 deg sides -> no bridges)
    zc, d = (ZT + ZB) / 2 - 2, 9.0
    for yc in DIAMOND_YS:
        dia = m3.Manifold.cube([PX0 - CAGE_X0 + 2, d * math.sqrt(2), d * math.sqrt(2)], center=True) \
            .rotate([45, 0, 0]).translate([(CAGE_X0 + PX0) / 2, yc, zc])
        w -= dia
    for yc in FINGER_YS:
        w += finger_x(yc)
    # ledge the box's X- edge sits on (bridged between the side frames)
    w += box(PX0 - 0.1, PX0 + 5.0, POST_Y0, PLATE_H - POST_Y0, ZB, ZL)
    return w


# ---------------------------------------------------------------- side frames (+/-Y)
def finger_y(xc, w=6.0):
    ztop = ZB - 3.5
    root_t = FINGER_T + 0.8
    blade = prism([(xc - w / 2, POST_Y1 - FINGER_T), (xc + w / 2, POST_Y1 - FINGER_T),
                   (xc + w / 2, POST_Y1), (xc - w / 2, POST_Y1)], PLATE_T - 0.01, ztop)
    root = hull2(box(xc - w / 2, xc + w / 2, POST_Y1 - root_t, POST_Y1, PLATE_T - 0.01, PLATE_T),
                            box(xc - w / 2, xc + w / 2, POST_Y1 - FINGER_T, POST_Y1, ztop - 0.01, ztop))
    prof = arc_bump(xc, POST_Y1, w / 2, Y_FINGER_REACH, '+y')
    bump = prism(prof, ztop - 11, ztop)
    lead = hull2(prism(prof, ztop - 11, ztop - 10.99),
                            box(xc - w / 2, xc + w / 2, POST_Y1 - 0.6, POST_Y1, ztop - 11 - Y_FINGER_REACH - 1, ztop - 11))
    return blade + root + bump + lead


def side_frame():
    """Built for the -Y side; mirrored for +Y."""
    xp0, xp1 = CAGE_X0, PX0 + 12.0               # closed-end corner post (holds a Y finger)
    s = box(xp0, xp1, POST_Y0, POST_Y1, 0.5, ZL)
    fx = (PX0 + xp1) / 2 + 0.5
    s -= box(fx - 3.8, fx + 3.8, POST_Y0 - 1, POST_Y1 + 1, PLATE_T, ZB - 1.5)
    s += finger_y(fx)
    # +X corner post (rigid, anchors the side bar)
    s += box(XPOST[0], XPOST[1], POST_Y0, POST_Y1, 0.5, ZL)
    # bottom side bar + ledge the box's short (port) edge rests on (bridge between posts)
    s += box(CAGE_X0, XPOST[1], POST_Y0, POST_Y1 + 5.0, ZB, ZL)
    return s


# ---------------------------------------------------------------- latch arms (+X end)
def latch_arm():
    """-Y side latch: a flat spring arm rooted on a post at ARM_ROOT_X, bending in Y
    (within the print layers = strong). Hook clicks in behind the box's +X face."""
    yi = POST_Y1 - 0.3            # inner face of the arm (rest position)
    yo = yi - ARM_T
    x_start = XPOST[1] + 1.0
    hook_tip_y = PY0 + HOOK_REACH
    bump_y = PY0 + ARM_BUMP_REACH
    prof = [(x_start, yo), (ARM_ROOT_X + 0.5, yo), (ARM_ROOT_X + 0.5, yi),
            # lead-in ramp (box arrives from +X)
            (PX1 + 1.0 + (hook_tip_y - yi) / math.tan(math.radians(28)), yi),
            (PX1 + 1.0, hook_tip_y), (PX1, hook_tip_y),
            # flat catch face at PX1
            (PX1, yi), (PX1 - 1.0, yi)]
    # side-press bump between x_start and the catch
    bx0, bx1 = x_start + 0.5, PX1 - 1.5
    n = 20
    for i in range(n + 1):
        t = i / n
        x = bx1 - (bx1 - bx0) * t
        y = yi + (bump_y - yi) * math.sin(math.pi * t)
        prof.append((x, y))
    prof.append((x_start, yi))
    arm = prism(prof, PLATE_T, ZL)
    # self-supporting underside: z >= PLATE_T + (ARM_ROOT_X - x) * SLOPE
    big = 400
    cut = xz_prism([(ARM_ROOT_X, PLATE_T - 0.01), (ARM_ROOT_X - big, PLATE_T - 0.01 + big * SLOPE),
                    (ARM_ROOT_X - big, -50), (ARM_ROOT_X + 10, -50), (ARM_ROOT_X + 10, PLATE_T - 0.01)],
                   -50, 300)
    arm -= cut
    # release ear on the outside, near the hook
    ex0 = PX1 + 6.0
    ear_z0 = max(ZB - 9, PLATE_T + (ARM_ROOT_X - ex0) * SLOPE)
    ear = box(ex0, ex0 + 5.0, POST_Y0 + 0.3, yo + 0.01, ear_z0, ZL)
    ear = hull2(ear, box(ex0, ex0 + 5.0, yo - 0.01, yo + 0.01, ear_z0 - (yo - POST_Y0), ZL))
    # root post
    post = box(ARM_ROOT_X, ARM_ROOT_X + 7.5, POST_Y0, POST_Y1 - 0.3, 0.5, ZL)
    return arm + ear + post


# ---------------------------------------------------------------- standoff rails
def rails():
    r = m3.Manifold()
    for yc in [YC - 50, YC, YC + 50]:
        x0, x1 = PX0 + 2, PX1 - 4
        prof = [(x0, PLATE_T - 0.01), (x1 - 4, PLATE_T - 0.01), (x1, PLATE_T - 0.01),
                (x1 - 4, ZT), (x0, ZT)]
        r += xz_prism(prof, yc - 1.25, yc + 1.25)
    return r


def build():
    part = plate() + end_wall() + rails()
    half = side_frame() + latch_arm()
    part = part + half + mirror_y(half)
    return part


def to_trimesh(man):
    mesh = man.to_mesh()
    tm = trimesh.Trimesh(np.asarray(mesh.vert_properties)[:, :3], np.asarray(mesh.tri_verts), process=False)
    return tm


def write_3mf(tm, path):
    import zipfile
    v = '\n'.join('<vertex x="%.5f" y="%.5f" z="%.5f"/>' % tuple(p) for p in tm.vertices)
    t = '\n'.join('<triangle v1="%d" v2="%d" v3="%d"/>' % tuple(f) for f in tm.faces)
    model = ('<?xml version="1.0" encoding="UTF-8"?>\n'
             '<model unit="millimeter" xml:lang="en-US" '
             'xmlns="http://schemas.microsoft.com/3dmanufacturing/core/2015/02">\n'
             '<resources><object id="1" name="pi_undercarriage_v2" type="model"><mesh>'
             '<vertices>%s</vertices><triangles>%s</triangles></mesh></object></resources>'
             '<build><item objectid="1"/></build></model>') % (v, t)
    ct = ('<?xml version="1.0" encoding="UTF-8"?><Types xmlns="http://schemas.openxmlformats.org/package/2006/content-types">'
          '<Default Extension="rels" ContentType="application/vnd.openxmlformats-package.relationships+xml"/>'
          '<Default Extension="model" ContentType="application/vnd.ms-package.3dmanufacturing-3dmodel+xml"/></Types>')
    rels = ('<?xml version="1.0" encoding="UTF-8"?><Relationships xmlns="http://schemas.openxmlformats.org/package/2006/relationships">'
            '<Relationship Target="/3D/3dmodel.model" Id="rel0" '
            'Type="http://schemas.microsoft.com/3dmanufacturing/2013/01/3dmodel"/></Relationships>')
    with zipfile.ZipFile(path, 'w', zipfile.ZIP_DEFLATED) as z:
        z.writestr('[Content_Types].xml', ct)
        z.writestr('_rels/.rels', rels)
        z.writestr('3D/3dmodel.model', model)


if __name__ == '__main__':
    import os
    here = os.path.dirname(os.path.abspath(__file__))
    man = build()
    print('manifold status', man.status(), 'genus', man.genus())
    tm = to_trimesh(man)
    print('pocket X %.2f  Y %.2f  Z %.2f' % (PX1 - PX0, PY1 - PY0, ZB - ZT))
    print('bounds', np.round(tm.bounds, 2), 'watertight', tm.is_watertight, 'volume cm3 %.1f' % (tm.volume / 1000))
    tm.export(os.path.join(here, 'undercarriage_v2.stl'))
    write_3mf(tm, os.path.join(here, 'undercarriage_v2.3mf'))
