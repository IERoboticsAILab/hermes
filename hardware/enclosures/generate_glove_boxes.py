#!/usr/bin/env python3
"""Open-tray enclosures for the H.E.R.M.E.S glove electronics. Units: mm.

Left box:  ESP32-on-PCB 55x32x15, IMU-on-PCB 15x35x18, flex-sensor breakout 17x27.
Right box: same minus the IMU.

Single-row layout along X, open top, USB notch at the ESP32 end, wire notch at the
flex-breakout end, strap flanges on both long sides. Geometry is a union of butted
axis-aligned boxes -- no CSG, no dependencies.

    python3 generate_glove_boxes.py     # writes the two STLs + runs the self-check
"""

import struct
import sys
from pathlib import Path

# --- printer / fit knobs (tune these, not the geometry below) ---------------
WALL = 2.0        # side wall thickness
FLOOR = 2.0       # floor thickness
CLEAR = 1.0       # per-side clearance across the box (Y)
GAP = 6.0         # wiring gap between adjacent boards (X)
MARGIN = 2.0      # gap between an end board and the end wall
INNER_Z = 20.0    # interior depth: tallest board (IMU, 18) + headroom
RIB_W, RIB_H = 2.0, 3.0     # divider rib between bays
USB_W, WIRE_W = 14.0, 12.0  # end-wall notch widths
FLANGE_OUT, SLOT_W, SLOT_L, SLOT_END = 12.0, 4.0, 25.0, 2.0  # strap flanges

# name, X size, Y size (Z only matters for INNER_Z)
ESP32 = ("esp32", 55.0, 32.0)
IMU = ("imu", 15.0, 35.0)
FLEX = ("flex", 17.0, 27.0)


def box(x0, y0, z0, x1, y1, z1):
    """12 outward-facing triangles for an axis-aligned box."""
    assert x1 > x0 and y1 > y0 and z1 > z0, (x0, y0, z0, x1, y1, z1)
    p = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    faces = [(0, 2, 1), (0, 3, 2), (4, 5, 6), (4, 6, 7),
             (0, 1, 5), (0, 5, 4), (3, 7, 6), (3, 6, 2),
             (0, 4, 7), (0, 7, 3), (1, 2, 6), (1, 6, 5)]
    return [(p[a], p[b], p[c]) for a, b, c in faces]


def flange(x0, y_out, y_in, z1):
    """Strap flange plate with a SLOT_L x SLOT_W slot, spanning y_out..y_in."""
    lo, hi = min(y_out, y_in), max(y_out, y_in)
    rail = (FLANGE_OUT - SLOT_W) / 2
    x1 = x0 + SLOT_L + 2 * SLOT_END
    return [
        (x0, lo, 0.0, x1, lo + rail, z1),                   # outer rail
        (x0, hi - rail, 0.0, x1, hi, z1),                   # inner rail
        (x0, lo + rail, 0.0, x0 + SLOT_END, hi - rail, z1),  # slot end
        (x1 - SLOT_END, lo + rail, 0.0, x1, hi - rail, z1),  # slot end
    ]


def tray(boards):
    """Return (list of boxes, LX, LY, LZ) for a single-row open tray."""
    inner_y = max(d for _, _, d in boards) + 2 * CLEAR
    inner_x = 2 * MARGIN + sum(w for _, w, _ in boards) + GAP * (len(boards) - 1)
    lx, ly, lz = inner_x + 2 * WALL, inner_y + 2 * WALL, FLOOR + INNER_Z

    b = [(0.0, 0.0, 0.0, lx, ly, FLOOR),                       # floor
         (0.0, 0.0, FLOOR, lx, WALL, lz),                      # long wall -Y
         (0.0, ly - WALL, FLOOR, lx, ly, lz)]                  # long wall +Y

    # end walls, each split around a full-height notch (no lid, so no bridging)
    for x0, x1, notch in ((0.0, WALL, USB_W), (lx - WALL, lx, WIRE_W)):
        b += [(x0, WALL, FLOOR, x1, (ly - notch) / 2, lz),
              (x0, (ly + notch) / 2, FLOOR, x1, ly - WALL, lz)]

    # divider ribs at the centre of each wiring gap
    x = WALL + MARGIN
    for i, (_, w, _) in enumerate(boards):
        x += w
        if i < len(boards) - 1:
            c = x + GAP / 2
            b.append((c - RIB_W / 2, WALL, FLOOR, c + RIB_W / 2, ly - WALL, FLOOR + RIB_H))
            x += GAP

    # two strap stations near the ends, flanged on both long sides
    plate = SLOT_L + 2 * SLOT_END
    for x0 in (10.0, lx - 10.0 - plate):
        b += flange(x0, -FLANGE_OUT, 0.0, FLOOR)
        b += flange(x0, ly + FLANGE_OUT, ly, FLOOR)
    return b, lx, ly, lz


def write_stl(path, boxes, dy):
    tris = [t for x0, y0, z0, x1, y1, z1 in boxes
            for t in box(x0, y0 + dy, z0, x1, y1 + dy, z1)]
    with open(path, "wb") as f:
        f.write(b"H.E.R.M.E.S glove enclosure".ljust(80, b"\0"))
        f.write(struct.pack("<I", len(tris)))
        for a, c, d in tris:
            u = [c[i] - a[i] for i in range(3)]
            v = [d[i] - a[i] for i in range(3)]
            n = [u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0]]
            m = sum(i * i for i in n) ** 0.5 or 1.0
            f.write(struct.pack("<12fH", *[i / m for i in n], *a, *c, *d, 0))
    return len(tris)


def check(boxes, boards, lx, ly, lz):
    """No two solids may overlap, and every board must fit its bay."""
    for i, a in enumerate(boxes):
        for c in boxes[i + 1:]:
            if all(min(a[k + 3], c[k + 3]) - max(a[k], c[k]) > 1e-9 for k in range(3)):
                raise AssertionError(f"overlapping solids: {a} {c}")
    inner_y = ly - 2 * WALL
    for name, w, d in boards:
        assert d + 2 * CLEAR <= inner_y + 1e-9, f"{name} too deep for the tray"
        assert max(h for h in (15.0, 18.0)) <= INNER_Z, "walls shorter than the boards"
    assert sum(w for _, w, _ in boards) + 2 * MARGIN + GAP * (len(boards) - 1) == lx - 2 * WALL
    assert lz == FLOOR + INNER_Z


if __name__ == "__main__":
    out = Path(__file__).parent
    for label, boards in (("left", [ESP32, IMU, FLEX]), ("right", [ESP32, FLEX])):
        boxes, lx, ly, lz = tray(boards)
        check(boxes, boards, lx, ly, lz)
        n = write_stl(out / f"glove_box_{label}.stl", boxes, FLANGE_OUT)
        print(f"{label:5s} {lx:.0f} x {ly:.0f} x {lz:.0f} mm body "
              f"({ly + 2 * FLANGE_OUT:.0f} mm wide over flanges), {n} triangles")
    print("self-check ok", file=sys.stderr)
