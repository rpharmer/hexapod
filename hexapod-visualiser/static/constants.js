export const LEG_ORDER = ["LF", "LM", "LR", "RF", "RM", "RR"];

export function deg2rad(v) {
  return (v * Math.PI) / 180;
}

export const MOUNT_ANGLES = {
  LF: deg2rad(40),
  LM: deg2rad(90),
  LR: deg2rad(140),
  RF: deg2rad(-40),
  RM: deg2rad(-90),
  RR: deg2rad(-140),
};

export const DEFAULT_GEOMETRY = {
  coxa: 35,
  femur: 70,
  tibia: 110,
  body_radius: 60,
};

export const DEFAULT_ANGLES_DEG = {
  LF: [0, 10, -25],
  LM: [0, 10, -25],
  LR: [0, 10, -25],
  RF: [0, 10, -25],
  RM: [0, 10, -25],
  RR: [0, 10, -25],
};

export const STALE_AFTER_MS = 1200;
export const ROLLING_LIMIT = 8;

export const CAMERA_PRESETS = {
  reset: { yaw: deg2rad(45), pitch: deg2rad(30), zoom: 1, panX: 0, panY: 0 },
  top: { yaw: deg2rad(0), pitch: deg2rad(-89), zoom: 1, panX: 0, panY: 0 },
  front: { yaw: deg2rad(0), pitch: deg2rad(8), zoom: 1, panX: 0, panY: 0 },
  side: { yaw: deg2rad(90), pitch: deg2rad(8), zoom: 1, panX: 0, panY: 0 },
};

export const CAMERA_BOUNDS = {
  zoomMin: 0.35,
  zoomMax: 3.5,
  pitchMin: deg2rad(-89),
  pitchMax: deg2rad(89),
  orbitSensitivity: 0.0085,
  panSensitivity: 1.0,
};
