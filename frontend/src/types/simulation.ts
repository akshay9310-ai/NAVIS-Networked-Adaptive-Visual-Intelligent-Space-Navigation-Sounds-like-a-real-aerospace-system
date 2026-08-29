/**
 * NAVIS Frontend TypeScript Interface Definitions
 */

export interface RoverState {
  x: number;
  y: number;
  vx: number;
  vy: number;
  speed: number;
  accel: number;
  heading_rad: number;
  heading_deg: number;
  angular_velocity: number;
  battery_pct: number;
  wheel_slip: number;
  wheel_speed_left: number;
  wheel_speed_right: number;
  total_distance: number;
  mission_status: 'IDLE' | 'NAVIGATING' | 'HAZARD_AVOIDANCE' | 'OUTAGE_FALLBACK' | 'TARGET_REACHED';
  target_pos: [number, number];
  dist_to_target: number;
}

export interface SatelliteState {
  sat_id: string;
  name: string;
  altitude_km: number;
  fov_radius_m: number;
  gx: number;
  gy: number;
  elevation_deg: number;
  distance_to_rover_m: number;
  is_visible: boolean;
  pnt_available: boolean;
  comm_available: boolean;
  imaging_available: boolean;
  battery_pct: number;
  current_task: 'PNT' | 'IMAGING' | 'COMMUNICATION' | 'IDLE';
  is_outage: boolean;
  score: number;
  decision_reason: string;
  capabilities: string[];
}

export interface CovarianceEllipse {
  semi_major_m: number;
  semi_minor_m: number;
  angle_rad: number;
  angle_deg: number;
}

export interface EKFState {
  est_x: number;
  est_y: number;
  est_vx: number;
  est_vy: number;
  est_speed: number;
  est_heading_rad: number;
  est_heading_deg: number;
  uncertainty_m: number;
  fallback_active: boolean;
  pnt_active: boolean;
  covariance_ellipse: CovarianceEllipse;
  bias_estimates: {
    b_ax: number;
    b_ay: number;
    b_omega: number;
  };
}

export interface SensorSuiteState {
  pnt: {
    active: boolean;
    x: number | null;
    y: number | null;
    accuracy_m: number | null;
    hdop: number | null;
  };
  imu: {
    active: boolean;
    ax: number;
    ay: number;
    omega: number;
    bias_ax: number;
    bias_ay: number;
    bias_omega: number;
  };
  odometry: {
    active: boolean;
    speed: number;
    distance_step: number;
    slip_pct: number;
  };
  visual_odometry: {
    active: boolean;
    dx: number;
    dy: number;
    dtheta: number;
    feature_confidence: number;
  };
}

export interface TrajectoryPoint {
  t: number;
  x: number;
  y: number;
}

export interface TrajectoryPrediction {
  predicted_trajectory: TrajectoryPoint[];
  horizon_sec: number;
  confidence_pct: number;
  mse_residual?: number;
  model_type?: string;
}

export interface AISchedulerState {
  selected_satellite?: string;
  selected_task?: string;
  reason_summary?: string;
  scores?: Record<string, number>;
  sub_scores?: Record<string, {
    visibility: number;
    rover_need: number;
    imaging_val: number;
    comm_need: number;
    resources: number;
  }>;
  explanations?: Record<string, string[]>;
  assigned_tasks?: Record<string, string>;
}

export interface HazardAlert {
  hazard_id: string;
  type: string;
  confidence_pct: number;
  distance_m: number;
  location: [number, number];
  source: string;
  timestamp: number;
}

export interface PerformanceMetricsState {
  current_position_error_m: number;
  avg_position_error_m: number;
  max_position_error_m: number;
  current_uncertainty_m: number;
  avg_uncertainty_m: number;
  max_uncertainty_m: number;
  coverage_pct: number;
  utilization_pct: number;
  route_deviation_m: number;
  active_satellites_count: number;
  visible_satellites_count: number;
  position_error_instant_m?: number;
}

export interface MissionLogEntry {
  id: number;
  time: string;
  met: string;
  category: string;
  message: string;
  level: 'info' | 'warning' | 'danger' | 'success';
}

export interface DemoMissionStatus {
  is_active: boolean;
  demo_time_s: number;
  current_stage_idx: number;
  stages: Array<{
    id: string;
    name: string;
    desc: string;
  }>;
}

export interface SimulationFullState {
  sim_time: number;
  is_running: boolean;
  speed_multiplier: number;
  active_mode: 'MODE_A' | 'MODE_B' | 'MODE_C';
  mode_meta: {
    name: string;
    tag: string;
    description: string;
  };
  rover: RoverState;
  satellites: SatelliteState[];
  ekf: EKFState;
  sensors: SensorSuiteState;
  metrics: PerformanceMetricsState;
  trajectory_prediction: TrajectoryPrediction;
  ai_scheduler: AISchedulerState;
  hazard_alert?: {
    hazard_risk_level: number;
    active_alert: HazardAlert | null;
    needs_replan: boolean;
    detected_count: number;
  };
  routes: {
    active_path: Array<[number, number]>;
    current_planned: Array<[number, number]>;
    old_avoided_path: Array<[number, number]>;
    breadcrumb_history: Array<{ x: number; y: number }>;
  };
  demo_mission: DemoMissionStatus;
  logs: MissionLogEntry[];
  terrain_summary: {
    width_m: number;
    height_m: number;
    start_pos: [number, number];
    target_pos: [number, number];
    injected_hazards: Array<{
      id: string;
      x: number;
      y: number;
      radius: number;
      type: string;
    }>;
    craters: Array<{
      x: number;
      y: number;
      radius: number;
      depth: number;
    }>;
    rock_fields: Array<{
      x: number;
      y: number;
      radius: number;
    }>;
  };
}

export interface BenchmarkComparisonResult {
  comparison_matrix: Record<string, {
    name: string;
    tag: string;
    position_error_avg_m: number;
    position_error_max_m: number;
    uncertainty_avg_m: number;
    coverage_pct: number;
    satellite_utilization_pct: number;
    route_deviation_pct: number;
    distance_traveled_m: number;
    hazard_detected: boolean;
    hazard_collided: boolean;
    mission_success: boolean;
    status_label: string;
  }>;
  best_mode: string;
  evaluation_seed: number;
  summary: string;
}
