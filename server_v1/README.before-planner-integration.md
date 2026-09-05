# LAAS Parking Server V1

This folder is the non-destructive starting point for the new parking server. It does not modify the current Raspberry Pi runtime and does not yet implement Hybrid A*.

## Frozen in V1

- Map frame: bottom-left origin, +X right, +Y up, CCW-positive yaw.
- Units: metres and radians.
- Parking slots: P_B1, P_B2, P_T1, P_T2.
- Slot states: UNKNOWN, FREE, OCCUPIED.
- Vehicle planning reference point: rear axle center.
- Pi/server transport contract: TCP + NDJSON.
- Server chooses the target among FREE slots.
- Pi retains local safety override.

## Intentionally unresolved

- Physical wheelbase verification (software baseline is 0.2515 m; conflicting reported value is 0.1895 m).
- Rear and front overhang.
- True maximum front-wheel steering angle.
- Final goal pose of each parking slot.
- Hybrid A* grid/yaw resolution, motion step, penalties, collision margin, Reeds-Shepp trigger and goal tolerances.
- Network timeout thresholds.

## First transport test

Run the server on the future server machine:

```bash
python3 server_stub.py --port <PORT>
```

Then send a single NDJSON `parking_status` message from a test client. Do not connect this stub to vehicle actuation yet.

## Next implementation milestone

1. Physically verify vehicle geometry.
2. Freeze four slot goal poses using rear-axle-center coordinates.
3. Add static occupancy grid / collision map.
4. Implement Hybrid A* as a server-side module.
5. Add trajectory validation.
6. Add Pi trajectory receiver and reverse-capable parking tracker behind a disabled feature flag.
7. Only after simulation/bench validation, integrate with STM32 actuation.
