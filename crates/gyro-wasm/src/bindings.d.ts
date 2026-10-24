/** Aircraft axes x forward, y right, z down; SI units throughout. */
export type Vec3 = [number, number, number];
/** Hamilton xyzw quaternion mapping local component coordinates to world. */
export type Quat = [number, number, number, number];
export type Preset = "Attitude" | "Directional" | "TurnIndicator";
export type DriveMode = "Powered" | "Coast";
export type Axis = "Roll" | "Pitch" | "Yaw";
export interface DriveState {
  mode: DriveMode;
  target_spin: number;
}
/** Turn: [theta, psi, 0]; attitude/DG: [alpha, beta, psi]. Spin is parent-relative. */
export interface JointState {
  angles: Vec3;
  rates: Vec3;
}
export interface HousingControllerConfig {
  position_gain: number;
  rate_time_constant: number;
  max_rate: number;
  max_acceleration: number;
}
export interface SimulationConfig {
  preset: Preset;
  rotor: { axial: number; transverse: number };
  gimbal_inertia: Vec3;
  inner_gimbal_inertia: Vec3;
  spring_stiffness: number;
  gimbal_damping: number;
  inner_spring_stiffness: number;
  inner_gimbal_damping: number;
  motor: { gain: number; torque_limit: number; spin_damping: number };
  housing_controller: HousingControllerConfig;
  nominal_axial_momentum: number;
  step_seconds: number;
}
export interface YawRateProfile {
  rate_rad_s: number;
  ramp_seconds: number;
  hold_seconds: number;
  initial_yaw_rad: number;
}
export interface ExcursionProfile {
  axis: Axis;
  angle_rad: number;
  move_seconds: number;
  hold_seconds: number;
}
export type HousingMotion =
  | { Fixed: { orientation: Quat } }
  | { YawRate: YawRateProfile }
  | { Excursion: ExcursionProfile }
  | { Manual: { orientation: Quat; angular_velocity: Vec3; target: Quat } };
export interface InitialState {
  joints: JointState;
  drive: DriveState;
  housing: HousingMotion;
  heading_offset: number;
}
export interface Definition {
  config: SimulationConfig;
  initial: InitialState;
}
export interface HousingSample {
  orientation: Quat;
  angular_velocity: Vec3;
  angular_acceleration: Vec3;
}
export type Maneuver =
  | { Yaw: { rate_rad_s: number; ramp_seconds: number; hold_seconds: number } }
  | { Excursion: ExcursionProfile };
export type Command =
  | "StopHousing"
  | { SetHousingTarget: Quat }
  | { StartManeuver: Maneuver }
  | { SetTargetSpin: number }
  | { SetDriveMode: DriveMode }
  | { SetHeadingOffset: number };
export interface CommandRecord {
  step_index: number;
  sequence: number;
  command: Command;
}
export interface Replay {
  version: number;
  config: SimulationConfig;
  initial: InitialState;
  commands: CommandRecord[];
  end_step: number;
}
export type InstrumentReading =
  | { Attitude: { pitch: number; bank: number | null } }
  | { Directional: { heading: number } }
  | { TurnIndicator: { deflection: number; indicated_rate: number } };
export interface Diagnostics {
  kinetic_energy: number;
  spring_energy: number;
  axial_momentum: number;
  absolute_axial_speed: number;
  spindle_direction: Vec3;
  rotor_angular_momentum: Vec3;
  assembly_angular_momentum: Vec3;
  housing_torque: Vec3;
  motor_torque: number;
  spin_loss_torque: number;
  housing_power: number;
  motor_power: number;
  dissipated_power: number;
  mass_condition: number;
  generalized_momenta: Vec3;
  near_axis_alignment: boolean;
  poorly_conditioned: boolean;
}
export interface Work {
  housing: number;
  motor: number;
  dissipated: number;
}
export interface Snapshot {
  preset: Preset;
  step_index: number;
  time_seconds: number;
  joints: JointState;
  drive: DriveState;
  housing: HousingSample;
  component_orientations: [Quat, Quat, Quat];
  housing_target: Quat | null;
  housing_rate_exceeded: boolean;
  heading_offset: number;
  instrument: InstrumentReading;
  diagnostics: Diagnostics;
  work: Work;
  energy_balance_residual: number;
}
export interface AdvanceResult {
  previous: Snapshot;
  current: Snapshot;
  steps_completed: number;
}
export interface SimulationError {
  kind: string;
  message: string;
  step: number | null;
}
