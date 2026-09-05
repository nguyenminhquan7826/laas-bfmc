# LAAS Parking Protocol V1

Transport: TCP. Framing: UTF-8 NDJSON; exactly one JSON object per line.
All distances are metres. All angles are radians. Map frame is `map`.

V1 is currently an OFFLINE/INTEGRATION protocol. A returned trajectory must not
be sent to actuators until full vehicle footprint and final vehicle geometry are
physically verified.

## Parking session state machine

Server V1 maintains one parking session with these states:

```text
IDLE
  -> WAITING_INPUT
  -> PLANNING
  -> TRAJECTORY_READY
  -> EXECUTING
       -> PAUSED
       -> REPLAN
       -> COMPLETED
```

A trajectory is never resumed blindly after a safety pause. `SAFETY_CLEARED`
causes a fresh replan from the latest accepted pose and parking status. If the
currently targeted slot changes from `FREE` to `OCCUPIED` or `UNKNOWN`, the
active trajectory is invalidated and the Server replans to another `FREE` slot.
A newly discovered better slot does not by itself interrupt an executing plan.

ACK and trajectory messages include a `session` object with `session_id`,
`state`, `active_trajectory_id`, `target_slot`, `replan_count`, and reason fields.


## Localization -> Server: vehicle_pose

```json
{"type":"vehicle_pose","version":1,"seq":151,"timestamp_ms":1787730000100,"map_id":"map_v1","source":"MOCK_LOCALIZATION","pose":{"x_m":1.30,"y_m":0.7511,"yaw_rad":0.0}}
```

`pose` is the rear-axle-center reference pose used by Hybrid A*.
During desktop tests `source` may be `MOCK_LOCALIZATION`. Later the server sensor
fusion/localization module should publish this same state internally.

## Pi -> Server: parking_status

```json
{"type":"parking_status","version":1,"seq":152,"timestamp_ms":1787730000123,"map_id":"map_v1","slots":[{"id":"P_B1","state":"OCCUPIED","confidence":0.96},{"id":"P_B2","state":"FREE","confidence":0.91},{"id":"P_T1","state":"UNKNOWN","confidence":0.45},{"id":"P_T2","state":"FREE","confidence":0.95}],"objects":[]}
```

Slot state is exactly one of `UNKNOWN`, `FREE`, `OCCUPIED`.
Server slot selection MUST consider only `FREE` slots. `UNKNOWN` is never implicitly free.

Optional object evidence:

```json
{"class":"car","confidence":0.94,"relative_x_m":1.72,"relative_y_m":-0.54,"associated_slot":"P_B1"}
```

A valid `parking_status` currently triggers planning automatically when planning
mode is enabled.

## Client -> Server: plan_request

Optional explicit request to plan using the latest accepted pose and parking status:

```json
{"type":"plan_request","version":1,"seq":153,"timestamp_ms":1787730000130,"map_id":"map_v1"}
```

This is mainly useful for integration tests and later replanning. The same
freshness and trajectory-validation guards apply as for automatic planning.

## Server -> Pi: trajectory

```json
{"type":"trajectory","version":1,"trajectory_id":37,"source_seq":152,"map_id":"map_v1","target_slot":"P_B2","reference_point":"rear_axle_center","goal_mode":"REAR_AXLE_AT_SLOT_CENTER_PROXY","input_age_ms_at_plan_start":{"pose":12.4,"parking_status":0.3},"validation":"PASS","prototype_warning":"OFFLINE_ONLY_FULL_VEHICLE_FOOTPRINT_NOT_VERIFIED","points":[{"x_m":1.30,"y_m":0.7511,"yaw_rad":0.0,"v_ref_mps":0.10,"direction":"FORWARD"},{"x_m":1.40,"y_m":0.7511,"yaw_rad":0.0,"v_ref_mps":0.10,"direction":"FORWARD"}]}
```

`direction` is `FORWARD` or `REVERSE`.
`v_ref_mps` is signed consistently with direction: positive for `FORWARD`, negative for `REVERSE`.
The Pi trajectory tracker must use the same rear-axle-center reference point as Hybrid A*.

A `FORWARD` terminal correction is valid if it improves the final parking pose.
Parking success is defined by final pose quality, not by the final motion direction.

The Server sends a trajectory only after V1 sanity validation passes. Current
checks include finite values, correct map/reference point, FREE target slot,
drivable-area membership, OCCUPIED/UNKNOWN slot avoidance, valid direction,
speed limit/sign consistency, path continuity, and replay of each Hybrid A*
motion primitive.

## Server -> Client: planning_result

Used when a valid trajectory cannot safely be returned.

```json
{"type":"planning_result","version":1,"map_id":"map_v1","source_seq":152,"status":"STALE_INPUT","reason":"pose_stale","pose_age_ms":2410.2,"parking_age_ms":300.1}
```

Current statuses include:

- `WAITING_FOR_INPUT`
- `STALE_INPUT`
- `NO_FREE_SLOT`
- `NO_FEASIBLE_TRAJECTORY`
- `INPUT_CHANGED_DURING_PLANNING`
- `TRAJECTORY_REJECTED`

## Input freshness guard

Staleness uses **server monotonic receive time**, not the sender's wall-clock
`timestamp_ms`. This avoids requiring Pi and Server clocks to be synchronized.

Current prototype defaults:

- pose max age: `2000 ms`
- parking-status max age: `3000 ms`

They are configurable with `--pose-max-age-ms` and `--parking-max-age-ms` and are
NOT frozen safety requirements yet. The Server checks freshness before planning
and again before sending the resulting trajectory. If a newer pose or parking
status arrives while planning is running, the old plan is rejected instead of
being sent.

## Pi -> Server: safety_event

```json
{"type":"safety_event","version":1,"timestamp_ms":1787730000456,"trajectory_id":37,"event":"PEDESTRIAN_BLOCKING"}
```

Safety events currently include `PEDESTRIAN_BLOCKING`, `CRITICAL_OBSTACLE`,
`SERVER_TIMEOUT`, `TRAJECTORY_INVALID`, and `SAFETY_CLEARED`.
Local Pi safety override is authoritative: the Pi stops first and reports the event afterward.
`SAFETY_CLEARED` never resumes the old path directly; it requests a fresh replan.

## Pi -> Server: trajectory_status

```json
{"type":"trajectory_status","version":1,"trajectory_id":37,"status":"PAUSED","reason":"PEDESTRIAN_BLOCKING"}
```

Supported states: `RECEIVED`, `EXECUTING`, `PAUSED`, `COMPLETED`, `REJECTED`,
and `REPLAN_REQUESTED`. `REJECTED` and `REPLAN_REQUESTED` invalidate the active
trajectory and request a fresh plan.

## Client -> Server: session_query

```json
{"type":"session_query","version":1,"map_id":"map_v1"}
```

Server replies with:

```json
{"type":"session_status","version":1,"map_id":"map_v1","session":{"session_id":1,"state":"EXECUTING","active_trajectory_id":37,"target_slot":"P_B2","replan_count":0}}
```

## Server -> Client: ack

```json
{"type":"ack","version":1,"seq":152,"accepted":true,"reason":"ok"}
```

## Protocol rules

1. Every message contains `type` and `version`.
2. Version 1 uses metres and radians only.
3. Each TCP line is one complete JSON message.
4. Unknown message types are rejected, never guessed.
5. Map-dependent messages use `map_id=map_v1`.
6. `vehicle_pose`, `parking_status`, and `plan_request` require non-negative integer `seq` and `timestamp_ms`.
7. Trajectory points are ordered from current vehicle pose toward the parking goal.
8. A trajectory may contain forward and reverse segments.
9. `UNKNOWN` is never treated as `FREE`.
10. Network loss never disables the Pi local stop layer.
11. Server planning requires fresh vehicle pose and parking status.
12. V1 trajectories remain offline-only until vehicle footprint/overhang geometry is verified.
13. If the active target slot is no longer `FREE`, its trajectory is invalidated and replanning is requested.
14. A safety pause is controlled locally by the Pi; Server state is advisory/orchestration only.
15. `SAFETY_CLEARED` causes replanning; it does not authorize direct resume of the previous trajectory.
16. After `COMPLETED`, automatic parking-status updates do not start a new session; use `plan_request` with `new_session=true`.
