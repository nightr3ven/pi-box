# Pi Box – Nanuk 909 build notes

Regenerate everything with `python pibox_case.py` (needs `numpy trimesh manifold3d`). The Pi cage is imported from `undercarriage_v2.py`, so changes from your v2 test print carry over automatically.

## Parts (`parts/`, each already in print orientation, all ≤ 256 mm)

| Part | Print | Supports |
|---|---|---|
| `base_frame_L` / `_R` | Countersunk face down | None |
| `pi_module` | Keyboard-floor face down, cage up | Build plate only, if bridges sag |
| `battery_module` | Keyboard-floor face down | None |
| `lid_frame_L` / `_R` | Screen back plate down | Tree supports under the arms and tabs |

## Hardware
- 16 × #6 × ½" flat-head (Nanuk stock): base frame halves → case
- 8 × #6 × ½" flat-head: lid frame halves → lid bosses (the two centre bosses are unused)
- 18 × M3 × 10 pan-head self-tapping: modules → base frame posts (inserted from below)
- 2 × M3 × 16 + nuts: join the lid frame halves (the screen gets captured between them)
- Panel-mount USB-C socket + 2 × M3 (charge port on the deck; slot 12.4 × 6.4, holes 25 mm apart; edit `USBC` to suit yours)
- Optional: 0.5 mm VHB between the screen and the back plate

## Assembly order
1. Put the battery in the sleeve on `battery_module` (ports toward the hinge side). Screw the module to `base_frame_R`.
2. Slide the DeskPi into `pi_module` until it clicks. Screw it to `base_frame_L`.
3. Run cables through the slots into the hinge-side trough. Then screw both frame halves into the case.
4. Lay the keyboard in its pocket, with its cable through the notch in the divider.
5. Put the screen face-down in `lid_frame_L`, close `lid_frame_R` over it, and bolt the seam. Screw the frame to the lid bosses. Route the screen cables along the left channel to the hinge side and down into the trough, leaving a slack loop for the lid to open.

## Measure these, then edit the top of `pibox_case.py` and re-run
| What | Current value | Where it bites |
|---|---|---|
| Keyboard L × W × thickness | 279.4 × 132.1 × 20.3 (listing) | Clearance to the screen lip (now 4.3 mm); pocket length vs. the case's side bosses |
| Base floor (deepest) → underside of the bezel plate | 56.9 (drawing) | Keyboard floor height over the battery |
| Lid plate face → lid ceiling | 23.8 (drawing) | Screen tray depth |
| Gap between the base and lid plates when closed | 1.9 (guess) | Lid riser length |
| Case screw-boss diameter / lid ledge width | Ø10 / ≤ 6 mm (guess) | Riser and post positions near the walls |
| Screen port edge and location | Left short edge, middle (guess) | Port opening + cable channel |
| Keyboard cable exit | Back edge, ~116 mm from the left (guess) | Notch in the divider |

## Electrical notes
- The Anker 737 supplies 5 V / 3 A to the Pi 5. It runs, but the Pi limits its USB ports to 600 mA. Power the screen from a second battery port, not from the Pi's USB.
- The Pi's ports face the case's front and back walls with about 15 mm free. Use right-angle plugs (HDMI, USB-C) there and at the battery's rear ports.
- The Pi boots when power is applied. Its power button ends up under the keyboard, so shut down from the OS.
