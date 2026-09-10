export interface PoseMessage {
  source?: string;
  pose?: { x_m: number; y_m: number; yaw_rad: number };
  server_receive_age_ms?: number;
}

export interface ParkingMessage {
  slots?: Array<{ id: string; state: string; confidence: number }>;
}

export interface RuntimeMessage {
  operating_mode?: string;
  tracker?: {
    valid: boolean;
    goal_reached: boolean;
    trajectory_id: number;
    nearest_index: number;
    target_index: number;
    cross_track_error_m: number;
  };
  safety?: { evaluated: boolean; motion_allowed: boolean; reason: string };
  uart?: {
    rx_enabled: boolean;
    tx_enabled: boolean;
    telemetry_valid: boolean;
    telemetry_age_ms: number;
  };
  session_sync?: { hold: boolean; reason: string };
}

export interface SessionSnapshot {
  session_id?: number;
  state?: string;
  active_trajectory_id?: number | null;
  target_slot_id?: string | null;
  replan_count?: number;
  pause_reason?: string | null;
}

export interface VehicleStatus {
  vehicle_id: string;
  connected: boolean;
  stale: boolean;
  last_seen_age_ms: number | null;
  last_message_type?: string | null;
  message_count: number;
  pose?: PoseMessage | null;
  parking?: ParkingMessage | null;
  trajectory?: Record<string, unknown> | null;
  safety?: Record<string, unknown> | null;
  runtime?: RuntimeMessage | null;
  session?: SessionSnapshot | null;
}

export interface VehiclesResponse { vehicles: VehicleStatus[]; }

export interface MapMetadata {
  map_id: string;
  frame?: Record<string, unknown>;
  width_x_m: number;
  height_y_m: number;
  slots: Array<{
    id: string;
    row?: string;
    polygon_m?: number[][];
    center_m?: number[];
  }>;
}
