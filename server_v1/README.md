# LAAS Parking Server V1

Desktop/offline parking-server prototype for `map_v1`. It does not modify the
Raspberry Pi runtime and has no STM32/UART/actuator interface.

## Current pipeline

```text
vehicle_pose + parking_status
            ↓
      freshness guards
            ↓
 parking session state machine
            ↓
   FREE-slot selection / replan
            ↓
       Hybrid A*
            ↓
 trajectory sanity validator
            ↓
     TCP/NDJSON trajectory
```

## Implemented

- CAD-derived `map_v1` and `DRIVABLE_AREA V1`.
- Four parking slots: `P_B1`, `P_B2`, `P_T1`, `P_T2`.
- States: `UNKNOWN`, `FREE`, `OCCUPIED`; only `FREE` is selectable.
- Rear-axle-center Hybrid A* reference point.
- Forward and reverse motion primitives.
- TCP + NDJSON server protocol.
- Offline slot selector + Hybrid A*.
- Server-side input freshness guard using monotonic receive time.
- Guard against pose/parking state changing while planning is running.
- Trajectory sanity validation before transmission.
- Parking Session State Machine:
  `IDLE -> WAITING_INPUT -> PLANNING -> TRAJECTORY_READY -> EXECUTING/PAUSED -> REPLAN/COMPLETED`.
- Automatic replan if the active target slot stops being `FREE`.
- Safety pause mirroring for `PEDESTRIAN_BLOCKING`/`CRITICAL_OBSTACLE`.
- `SAFETY_CLEARED` triggers a fresh replan; the old trajectory is never blindly resumed.

## Prototype staleness defaults

```text
pose max age           = 2000 ms
parking-status max age = 3000 ms
```

These values are not final safety thresholds. They will be measured on the real
Pi/server LAN later.

## Desktop test

PowerShell 1:

```powershell
py server_stub.py --port 5000
```

PowerShell 2, basic planning:

```powershell
py mock_planning_client_v1.py --host 127.0.0.1 --port 5000 --scenario basic
```

Expected:

```text
[RESULT] PASS
```

State-machine + replan test:

```powershell
py mock_planning_client_v1.py --host 127.0.0.1 --port 5000 --scenario replan
```

The replan scenario checks:

```text
initial target P_B2
P_B2 becomes OCCUPIED
-> replan to P_T2
-> EXECUTING
-> PEDESTRIAN_BLOCKING
-> PAUSED
-> fresh pose while paused does not resume
-> SAFETY_CLEARED
-> fresh replan to P_T2
-> COMPLETED
```

Expected final line:

```text
[RESULT] REPLAN PASS
```

## Still intentionally unresolved

- Physical wheelbase verification.
- Rear/front overhang and exact rear-axle-to-body-center offset.
- True maximum front-wheel steering angle.
- Full vehicle footprint collision.
- Exact final parking goal in rear-axle coordinates.
- Final network timeout/staleness thresholds.

## Safety boundary

A `validation=PASS` trajectory means only that the current **offline V1** checks
passed. It is not authorization to drive the real vehicle. Full-body collision
and physical geometry must be verified before actuator integration.

The Pi remains authoritative for immediate STOP/HOLD. The Server state machine
coordinates planning/replanning but is not the realtime emergency-stop layer.
