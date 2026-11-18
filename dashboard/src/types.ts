export type DriveMode = "IDLE" | "MANUAL" | "AUTONOMOUS" | "ESTOP";

export type SafetyFault =
  | "NONE"
  | "COMM_LOSS_500MS"
  | "ESTOP_SOFTWARE"
  | "ESTOP_HARDWARE"
  | "TILT_ROLLOVER"
  | "LOW_BATTERY";

export interface RescueTarget {
  id: string;
  x: number;
  y: number;
  label: string;
  detected: boolean;
}

export interface GridSnapshot {
  width: number;
  height: number;
  res: number;
  origin_x: number;
  origin_y: number;
  cells: number[];
}

export interface WorldSnapshot {
  bounds: [number, number];
  walls: [number, number, number, number][];
  obstacles: { x: number; y: number; w: number; h: number; label: string }[];
  targets: RescueTarget[];
}

export interface TelemetryState {
  t_ms: number;
  wall_ts: number;
  mode: DriveMode;
  fault: SafetyFault;
  motors_enabled: boolean;
  comm_loss_injected: boolean;
  watchdog_ms: number;
  watchdog_timeout_ms: number;
  avoidance: boolean;
  pose: {
    x: number;
    y: number;
    theta: number;
    v: number;
    w: number;
  };
  true_pose: {
    x: number;
    y: number;
    theta: number;
  };
  rpm: {
    left: number;
    right: number;
  };
  tof_m: number[];
  tof_angles_rad: number[];
  imu: {
    roll: number;
    pitch: number;
    yaw: number;
  };
  battery: {
    v: number;
    soc: number;
    low_warn: boolean;
  };
  waypoint: { x: number; y: number } | null;
  map_stats: {
    explored_cells: number;
    occupied_cells: number;
    free_cells: number;
    explored_pct: number;
  };
  targets: RescueTarget[];
  grid?: GridSnapshot;
  world?: WorldSnapshot;
}

export type OutboundCommand =
  | { type: "cmd_vel"; linear: number; angular: number }
  | { type: "waypoint"; x: number; y: number }
  | { type: "mode"; mode: DriveMode }
  | { type: "estop" }
  | { type: "clear_estop" }
  | { type: "comm_loss"; active: boolean }
  | { type: "heartbeat" }
  | { type: "reset" };
