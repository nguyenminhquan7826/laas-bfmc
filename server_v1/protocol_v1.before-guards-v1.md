# LAAS Parking Protocol V1

Transport: TCP. Framing: UTF-8 NDJSON; exactly one JSON object per line.
All distances are metres. All angles are radians. Map frame is `map`.

V1 is currently an OFFLINE/INTEGRATION protocol. A returned trajectory must not
be sent to actuators until full vehicle footprint and final vehicle geometry are
physically verified.

## Localization -> Server: vehicle_pose

```json
{"type":"vehicle_pose","version":1,"seq":151,"timestamp_ms":1787730000100,"map_id":"map_v1","source":"MOCK_LOCALIZATION","pose":{"x_m":1.30,"y_m":0.7511,"yaw_rad":0.0}}
```

`pose` is the rear-axle-center reference pose used by Hybrid A*.
During desktop tests `source` may be `MOCK_LOCALIZATION`. Later the server sensor
fusion/localization module should publish this same message/state internally.

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

## Server -> Pi: trajectory

```json
{"type":"trajectory","version":1,"trajectory_id":37,"source_seq":152,"map_id":"map_v1","target_slot":"P_B2","reference_point":"rear_axle_center","goal_mode":"REAR_AXLE_AT_SLOT_CENTER_PROXY","prototype_warning":"OFFLINE_ONLY_FULL_VEHICLE_FOOTPRINT_NOT_VERIFIED","points":[{"x_m":1.30,"y_m":0.7511,"yaw_rad":0.0,"v_ref_mps":0.10,"direction":"FORWARD"},{"x_m":1.40,"y_m":0.7511,"yaw_rad":0.0,"v_ref_mps":0.10,"direction":"FORWARD"}]}
```

`direction` is `FORWARD` or `REVERSE`. The Pi trajectory tracker must use the
same rear-axle-center reference point as Hybrid A*.

A `FORWARD` terminal correction is valid if it improves the final parking pose.
Parking success is defined by final pose quality, not by the final motion direction.

## Server -> Client: planning_result

Used when a valid trajectory cannot yet be returned.

```json
{"type":"planning_result","version":1,"map_id":"map_v1","source_seq":152,"status":"WAITING_FOR_POSE","reason":"no_vehicle_pose"}
```

Current statuses include:

- `WAITING_FOR_POSE`
- `NO_FREE_SLOT`
- `NO_FEASIBLE_TRAJECTORY`

## Pi -> Server: safety_event

```json
{"type":"safety_event","version":1,"timestamp_ms":1787730000456,"trajectory_id":37,"event":"PEDESTRIAN_BLOCKING"}
```

Safety events include at least `PEDESTRIAN_BLOCKING`, `CRITICAL_OBSTACLE`,
`SERVER_TIMEOUT`, and `TRAJECTORY_INVALID`.
Local Pi safety override is authoritative: the Pi stops first and reports the event afterward.

## Pi -> Server: trajectory_status

```json
{"type":"trajectory_status","version":1,"trajectory_id":37,"status":"PAUSED","reason":"PEDESTRIAN_BLOCKING"}
```

Suggested states: `RECEIVED`, `EXECUTING`, `PAUSED`, `COMPLETED`, `REJECTED`.

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
6. Trajectory points are ordered from current vehicle pose toward the parking goal.
7. A trajectory may contain forward and reverse segments.
8. `UNKNOWN` is never treated as `FREE`.
9. Network loss never disables the Pi local stop layer.
10. Server planning currently requires both a current vehicle pose and a valid parking status.
11. Timing/staleness thresholds are not frozen yet; they will be measured on the real LAN.
12. V1 trajectories remain offline-only until vehicle footprint/overhang geometry is verified.
