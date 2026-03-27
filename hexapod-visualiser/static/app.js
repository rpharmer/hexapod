import {
  CAMERA_BOUNDS,
  CAMERA_PRESETS,
  DEFAULT_ANGLES_DEG,
  DEFAULT_GEOMETRY,
} from "./constants.js";
import { createSceneRenderer } from "./scene-renderer.js";
import { installInputController } from "./input-controller.js";
import { applyStatePayload, createTransport } from "./transport.js";

const statusEl = document.getElementById("status");
const metaEl = document.getElementById("meta");
const rollingMetricsEl = document.getElementById("rolling-metrics");
const canvas = document.getElementById("scene");
const ctx = canvas.getContext("2d");
const viewButtons = document.querySelectorAll("[data-view]");

const modelRef = {
  current: {
    geometry: { ...DEFAULT_GEOMETRY },
    angles_deg: { ...DEFAULT_ANGLES_DEG },
    timestamp_ms: null,
    active_mode: null,
    active_fault: null,
    bus_ok: null,
    estimator_valid: null,
    loop_counter: null,
    voltage: null,
    current: null,
    dynamic_gait: null,
    autonomy_debug: null,
  },
};

const telemetry = {
  lastPayloadAtMs: 0,
  lastModelTimestampMs: null,
  rolling: [],
};

const camera = { ...CAMERA_PRESETS.reset };

function applyCameraPreset(name) {
  const preset = CAMERA_PRESETS[name];
  if (!preset) return;
  Object.assign(camera, preset);
}

viewButtons.forEach((button) => {
  button.addEventListener("click", () => {
    applyCameraPreset(button.dataset.view);
  });
});

installInputController(canvas, camera, CAMERA_BOUNDS);

const renderer = createSceneRenderer({
  canvas,
  ctx,
  camera,
  statusEl,
  metaEl,
  rollingMetricsEl,
  modelRef,
  telemetry,
});
renderer.start();

const wsProtocol = location.protocol === "https:" ? "wss" : "ws";
const transport = createTransport({
  websocketUrl: `${wsProtocol}://${location.host}/ws`,
  statusEl,
  onPayload: (payload) => {
    modelRef.current = applyStatePayload(modelRef.current, telemetry, payload);
  },
});
transport.start();
