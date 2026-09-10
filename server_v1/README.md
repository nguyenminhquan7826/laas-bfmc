# LAAS Parking Server V1

Parking and navigation prototype for `map_v1`. Step 3 adds a guarded migration
to client-owned Hybrid A*: the Raspberry Pi owns the operational map and local
parking plan, while the server issues high-level decisions and monitors status.
Neither server mode has an STM32/UART/actuator interface.

## Planning ownership modes

The stable default remains server-owned planning:

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

The new Step 3 staging mode is client-owned planning:

```text
server: parking status -> target/maneuver decision
                         ↓ navigation_decision + map hash
Pi:     verify local map -> local Hybrid A* -> local tracker/safety
                         ↓ read-only status/trajectory telemetry
server: monitoring dashboard + later intersection/roundabout decisions
```

The decision is sent early and may be cached by the Pi. Loss of the server must
not disable localization, the current local trajectory, or the immediate local
STOP/HOLD layer.

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
- BFMC-inspired Angular 18 monitoring dashboard, served by the read-only HTTP
  backend.
- Deterministic operational-map manifest and SHA-256 package identity.
- Feature-flagged `navigation_decision` contract for Pi-owned Hybrid A*.
- Standalone Pi local-planner worker with map verification and trajectory
  validation.

## Step 3: client-owned local planning

Keep the current working behavior with the default:

```powershell
py server_stub.py --port 5000 --monitor-port 5005 --planning-owner server
```

Exercise the new server decision boundary without enabling actuator output:

```powershell
py server_stub.py --port 5000 --monitor-port 5005 --planning-owner client
```

In client mode, the server sends `PARK_AT_SLOT` plus the selected slot and exact
`map_package_sha256`; it does not send a trajectory. The Pi accepts the decision
only when the hash matches its checked-in operational map. The local trajectory
uses `trajectory_id == decision_id`, which makes monitoring and acknowledgements
traceable across worker restarts.

Current Step 3A integration boundary:

- Server decision, map package, C++ decoding/expected-identity check,
  monitoring, and the standalone local Hybrid A* worker are implemented and
  tested. The worker performs the actual on-disk checksum verification.
- The C++ runtime deliberately enters `HOLD/LOCAL_PLANNER_PENDING` after accepting
  a decision. Starting the Python worker asynchronously, loading its trajectory
  into the C++ tracker, and reporting `READY/REJECTED` is Step 3B.
- Therefore use `--planning-owner server` for the present end-to-end vehicle run.
  Client mode is safe contract/integration testing only until Step 3B lands.

Run the Step 3 software tests from `server_v1`:

```powershell
py -m unittest tests.test_client_owned_planning tests.test_server_protocol
```

`map_manifest_v1.json` covers `map_v1.yaml`, `vehicle_v1.yaml`,
`planner_v1.yaml`, and `drivable_grid_v1.json`. Any operational-file change must
regenerate the manifest and update the expected hash in the Pi configuration;
otherwise planning fails closed.

## Step 13 monitoring dashboard and API

The monitoring service starts with the parking server on TCP port `5005` by
default. It has no command or actuator endpoint.

```text
GET /api/health
GET /api/vehicles
GET /api/vehicles/{vehicle_id}/status
GET /api/map
GET /api/map/reference
GET /api/events                    # Server-Sent Events
```

Example:

```powershell
py server_stub.py --port 5000 --monitor-port 5005
```

Build the dashboard once:

```powershell
cd dashboard/frontend
npm install
npm run build
cd ../..
```

Then open `http://127.0.0.1:5005/`. The dashboard exposes the latest
protocol-validated Encoder-IMU pose, parking slots, trajectory/session status,
tracker progress/error, safety gate, UART RX/TX policy, connection state, and
receive age. SSE updates the screen every 500 ms; REST polling is the fallback.

For frontend development, keep the Python server on port 5005 and run:

```powershell
cd dashboard/frontend
npm start
```

Then open `http://127.0.0.1:4200/`. The development build automatically uses
the monitoring API at the same host on port 5005.

The UI architecture follows the official Bosch Future Mobility Challenge
Angular dashboard, reduced to localization/parking monitoring. Attribution is
recorded in `dashboard/frontend/THIRD_PARTY_NOTICES.md`.

Use `--no-monitoring` only for a transport/planner test that does not need the
HTTP endpoint.

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

The Pi remains authoritative for immediate STOP/HOLD. In server-owned mode the
Server coordinates planning/replanning. In client-owned mode it supplies only
high-level navigation decisions and monitoring; it is never the realtime
emergency-stop layer.
