export type Availability = {
  move_xyz?: boolean;
  home_axis?: boolean;
  go_origin?: boolean;
  enable_axis?: boolean;
  set_zero?: boolean;
  set_axis_limits?: boolean;
  set_gripper?: boolean;
  clear_fault?: boolean;
  release_stall?: boolean;
  get_driver_status?: boolean;
  command_topic?: boolean;
  aux_servo?: boolean;
};

export type PalletizerState = {
  stamp_ms: number;
  ros: {
    domain_id: string;
    backend_node: string;
    palletizer_node: string;
  };
  connections: Record<string, boolean>;
  last_seen_age_ms: Record<string, number>;
  availability: Availability;
  axis_position_mm: number[];
  joint_states: {
    name: string[];
    position: number[];
    velocity: number[];
    effort: number[];
    stamp: { sec: number; nanosec: number };
  };
  motor_rpm: number[];
  status_raw: string;
  status: Record<string, unknown>;
  fault_state: string;
  last_action?: unknown;
};

export type BackendEvent = {
  type: string;
  state?: PalletizerState;
  action?: string;
  feedback?: Record<string, unknown>;
  result?: Record<string, unknown>;
  response?: Record<string, unknown>;
  service?: string;
  message?: string;
  data?: unknown;
};

export type MoveGoal = {
  x_mm: number;
  y_mm: number;
  z_mm: number;
  use_a: boolean;
  a_deg: number;
  speed_mm_s: number;
  accel_mm_s2: number;
  angular_speed_deg_s: number;
  angular_accel_deg_s2: number;
  tolerance_mm: number;
  angular_tolerance_deg: number;
  timeout_ms: number;
};

export const emptyState: PalletizerState = {
  stamp_ms: 0,
  ros: { domain_id: "", backend_node: "/palletizer_ui_backend", palletizer_node: "/palletizer_controller" },
  connections: {},
  last_seen_age_ms: {},
  availability: {},
  axis_position_mm: [0, 0, 0],
  joint_states: { name: [], position: [], velocity: [], effort: [], stamp: { sec: 0, nanosec: 0 } },
  motor_rpm: [0, 0, 0, 0, 0],
  status_raw: "",
  status: {},
  fault_state: "",
  last_action: undefined
};
