# Scoop Geometry Measurements

Date: 2026-04-07
Branch: `ben/full-scoop`

This file records the physical and CAD-derived measurements used to tune the
LitterBot scoop mechanism. Values should be updated as measurements are
confirmed on the real robot.

## Ramp

Measured triangle:

| Quantity | Value | Notes |
|---|---:|---|
| Ramp hypotenuse length | 14.39 in | User-provided |
| Ramp vertical rise | 13.19 in | User-provided |
| Ramp horizontal run | 5.75 in | User-provided |
| Ramp angle from horizontal | 66.4 deg | Derived from rise/run |

## Scoop Linkage

Current measurements:

| Quantity | Value | Notes |
|---|---:|---|
| Arm length | 14.69 in | From servo connection point to ramp connection point |
| Lead screw / carriage slide member length | 16.54 in | User-provided |
| Ramp connection point to ramp tip | 2.0 in | User-provided; along ramp piece hypotenuse |
| Scoop pivot to scoop lip | ~2.0 in | Tentative; appears to match ramp connection point to tip |
| Servo horn radius | TBD | Center of shaft to linkage hole |
| Linkage rod center-to-center length | TBD | |
| Servo shaft center to scoop pivot center | 0 in | Servo axis and arm rotation axis are the same |

## Stepper Waypoints

Pending measurement:

| Quantity | Value | Notes |
|---|---:|---|
| Home to first ramp contact | TBD steps | |
| Home to fully seated on ramp | TBD steps | |
| Home to pickup point | TBD steps | |
| Home to dump/bin retract point | TBD steps | |
| Total travel | TBD steps | |

## Servo Presets

Pending calibration:

| Preset | Value | Notes |
|---|---:|---|
| HOME | TBD | Safe stowed angle |
| APPROACH_START | TBD | First ramp-follow angle |
| APPROACH_END | TBD | Ramp-follow angle deeper into stroke, if needed |
| CAPTURE | TBD | Litter retention angle |
| DUMP | TBD | Over-bin dump angle |

## Notes

- The ramp is steep enough that servo coordination during extension will likely
  need either interpolation or multiple approach waypoints.
- Measure linkage geometry from CAD first, then confirm installed dimensions on
  the real robot.
