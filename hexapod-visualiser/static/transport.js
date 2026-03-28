export function createTransport({
  websocketUrl,
  statusEl,
  onPayload,
  reconnectDelayMs = 1000,
}) {
  let reconnectTimer = null;
  let ws = null;
  let shouldReconnect = true;

  function connect() {
    ws = new WebSocket(websocketUrl);

    ws.addEventListener("open", () => {
      statusEl.textContent = "Connected";
      statusEl.style.color = "#34d399";
    });

    ws.addEventListener("message", (event) => {
      try {
        onPayload(JSON.parse(event.data));
      } catch (err) {
        console.warn("invalid payload", err);
      }
    });

    ws.addEventListener("close", () => {
      statusEl.textContent = "Disconnected (retrying)";
      statusEl.style.color = "#f87171";
      ws = null;
      if (shouldReconnect) {
        reconnectTimer = setTimeout(connect, reconnectDelayMs);
      }
    });
  }

  return {
    start() {
      shouldReconnect = true;
      connect();
    },
    stop() {
      shouldReconnect = false;
      if (reconnectTimer !== null) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
      }
      if (ws) {
        ws.close();
        ws = null;
      }
    },
  };
}

export function applyStatePayload(model, telemetry, payload, nowMs = Date.now()) {
  telemetry.lastPayloadAtMs = nowMs;

  if (payload.type !== "state") return model;

  const payloadTimestampMs = typeof payload.timestamp_ms === "number" ? payload.timestamp_ms : null;
  const lastTimestampMs = Number.isFinite(telemetry.lastModelTimestampMs)
    ? telemetry.lastModelTimestampMs
    : Number.NEGATIVE_INFINITY;
  const hasStaleTimestamp = payloadTimestampMs !== null && payloadTimestampMs < lastTimestampMs;
  const hasEqualTimestamp = payloadTimestampMs !== null && payloadTimestampMs === lastTimestampMs;

  if (hasStaleTimestamp) {
    return model;
  }

  if (hasEqualTimestamp) {
    const payloadSampleId = payload.sample_id;
    const hasSampleId = typeof payloadSampleId === "string" || typeof payloadSampleId === "number";
    const lastSampleId = telemetry.lastModelSampleId;
    const canOrderBySampleId =
      hasSampleId && (typeof lastSampleId === "string" || typeof lastSampleId === "number");

    if (!canOrderBySampleId || payloadSampleId <= lastSampleId) {
      return model;
    }
  }

  const nextModel = {
    ...model,
    geometry: { ...model.geometry, ...(payload.geometry || {}) },
    angles_deg: { ...model.angles_deg, ...(payload.angles_deg || {}) },
    timestamp_ms: payload.timestamp_ms ?? model.timestamp_ms,
    active_mode: payload.active_mode ?? model.active_mode ?? null,
    active_fault: payload.active_fault ?? model.active_fault ?? null,
    bus_ok: payload.bus_ok ?? model.bus_ok ?? null,
    estimator_valid: payload.estimator_valid ?? model.estimator_valid ?? null,
    loop_counter: payload.loop_counter ?? model.loop_counter ?? null,
    voltage: payload.voltage ?? model.voltage ?? null,
    current: payload.current ?? model.current ?? null,
    dynamic_gait: payload.dynamic_gait ?? model.dynamic_gait ?? null,
    autonomy_debug: payload.autonomy_debug ?? model.autonomy_debug ?? null,
  };

  if (payloadTimestampMs !== null) {
    telemetry.lastModelTimestampMs = payloadTimestampMs;
  }
  if (typeof payload.sample_id === "string" || typeof payload.sample_id === "number") {
    telemetry.lastModelSampleId = payload.sample_id;
  }

  return nextModel;
}
