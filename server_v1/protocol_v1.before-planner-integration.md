# LAAS Parking Protocol V1

Transport: TCP. Framing: UTF-8 NDJSON; exactly one JSON object per line.
All distances are metres. All angles are radians. Map frame is `map`.

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
{"type":"trajectory","version":1,"trajectory_id":37,"map_id":"map_v1","target_slot":"P_B2","points":[{"x_m":2.31,"y_m":0.75,"yaw_rad":0.02,"v_ref_mps":0.10,"direction":"FORWARD"},{"x_m":2.49,"y_m":0.68,"yaw_rad":-0.20,"v_ref_mps":0.08,"direction":"REVERSE"}]}
```

`direction` is `FORWARD` or `REVERSE`. The Pi trajectory tracker must use the same rear-axle-center reference point as Hybrid A*.

## Pi -> Server: safety_event

```json
{"type":"safety_event","version":1,"timestamp_ms":1787730000456,"trajectory_id":37,"event":"PEDESTRIAN_BLOCKING"}
```

Safety events include at least `PEDESTRIAN_BLOCKING`, `CRITICAL_OBSTACLE`, `SERVER_TIMEOUT`, and `TRAJECTORY_INVALID`.
Local Pi safety override is authoritative: the Pi stops first and reports the event afterward.

## Pi -> Server: trajectory_status

```json
{"type":"trajectory_status","version":1,"trajectory_id":37,"status":"PAUSED","reason":"PEDESTRIAN_BLOCKING"}
```

Suggested states: `RECEIVED`, `EXECUTING`, `PAUSED`, `COMPLETED`, `REJECTED`.

## Server -> Pi: ack

```json
{"type":"ack","version":1,"seq":152,"accepted":true}
```

## Protocol rules

1. Every message contains `type` and `version`.
2. Version 1 uses metres and radians only.
3. Each TCP line is one complete JSON message.
4. Unknown message types are rejected, never guessed.
5. A trajectory from a different `map_id` is rejected.
6. Trajectory points must be ordered from current vehicle pose toward the parking goal.
7. A trajectory may contain forward and reverse segments.
8. Network loss never disables the Pi local stop layer.
9. Timing thresholds are intentionally not frozen in V1 yet; they will be measured on the real LAN before being made safety requirements.
