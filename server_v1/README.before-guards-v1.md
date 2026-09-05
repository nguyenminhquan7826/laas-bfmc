# LAAS Parking Server V1

This folder is the non-destructive server-side parking prototype. It does not
modify the current Raspberry Pi runtime and does not contain an actuator/STM32
interface.

## Current status

Implemented and desktop-tested:

- Map frame: bottom-left origin, +X right, +Y up, CCW-positive yaw.
- Units: metres and radians.
- CAD-derived map/parking geometry.
- Parking slots: `P_B1`, `P_B2`, `P_T1`, `P_T2`.
- Slot states: `UNKNOWN`, `FREE`, `OCCUPIED`.
- `UNKNOWN` is never treated as free.
- CAD-derived drivable-area grid with intermediate primitive sampling.
- Hybrid A* forward/reverse planning.
- Server-side FREE-slot selection.
- TCP + NDJSON transport.
- `vehicle_pose` input for desktop/mock localization.
- `parking_status` input.
- `trajectory` output.
- Explicit `planning_result` when no trajectory can be returned.
- Batch drivable-area tests.

## Safety limitation

Planner output is still OFFLINE ONLY.

Full vehicle footprint collision is not enabled because rear/front overhang and
final physical steering geometry have not yet been verified on the actual car.
The server has no UART, motor, servo, or STM32 output path.

## Desktop validation

Validate the semantic map:

```powershell
py validate_drivable_v1.py
```

Run Hybrid A* batch cases:

```powershell
py batch_hybrid_test_v1.py
```

Run one visual parking case:

```powershell
py offline_hybrid_test.py
start plan_debug.svg
```

## End-to-end server planning test

Terminal 1:

```powershell
py server_stub.py --port 5000
```

Terminal 2:

```powershell
py mock_planning_client_v1.py --host 127.0.0.1 --port 5000
```

The mock client sends:

1. one `vehicle_pose`,
2. one `parking_status`,
3. receives ACKs,
4. receives a Hybrid A* `trajectory` or an explicit `planning_result`.

A successful run writes `mock_trajectory_response.json`.

## Still unresolved before real vehicle use

- Physical wheelbase verification. Software baseline is 0.2515 m; an older
  conflicting value of 0.1895 m must still be resolved physically.
- Rear overhang and front overhang.
- Exact rear-axle-to-body-center offset.
- True maximum front-wheel steering angle.
- Full rectangular vehicle footprint collision.
- Final physical parking tolerance tuning.
- Network staleness/timeout thresholds.
- Pi reverse-capable trajectory tracker integration.

## Next milestone

After the desktop server test passes, add trajectory sanity validation and
server-side stale-pose/stale-parking-status guards. After that, when the Pi is
available, connect only the TCP receiver first while keeping actuator output
disabled.
