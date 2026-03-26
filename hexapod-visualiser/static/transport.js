export function createTransport({
  websocketUrl,
  statusEl,
  onPayload,
  reconnectDelayMs = 1000,
}) {
  let reconnectTimer = null;

  function connect() {
    const ws = new WebSocket(websocketUrl);

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
      reconnectTimer = setTimeout(connect, reconnectDelayMs);
    });
  }

  return {
    start() {
      connect();
    },
    stop() {
      if (reconnectTimer !== null) {
        clearTimeout(reconnectTimer);
        reconnectTimer = null;
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

  if (hasStaleTimestamp) {
    return model;
  }

  const nextModel = {
    ...model,
    geometry: { ...model.geometry, ...(payload.geometry || {}) },
    angles_deg: { ...model.angles_deg, ...(payload.angles_deg || {}) },
    timestamp_ms: payload.timestamp_ms ?? model.timestamp_ms,
  };

  if (payloadTimestampMs !== null) {
    telemetry.lastModelTimestampMs = payloadTimestampMs;
  }

  return nextModel;
}
