function clamp(value, min, max) {
  return Math.max(min, Math.min(max, value));
}

export function installInputController(canvas, camera, bounds) {
  const interaction = {
    pointerId: null,
    mode: null,
    lastX: 0,
    lastY: 0,
  };

  canvas.addEventListener("contextmenu", (event) => {
    event.preventDefault();
  });

  canvas.addEventListener("pointerdown", (event) => {
    interaction.pointerId = event.pointerId;
    interaction.lastX = event.clientX;
    interaction.lastY = event.clientY;

    if (event.button === 2 || event.button === 1 || event.shiftKey) {
      interaction.mode = "pan";
    } else {
      interaction.mode = "orbit";
    }

    canvas.setPointerCapture(event.pointerId);
  });

  canvas.addEventListener("pointermove", (event) => {
    if (interaction.pointerId !== event.pointerId || !interaction.mode) return;

    const dx = event.clientX - interaction.lastX;
    const dy = event.clientY - interaction.lastY;
    interaction.lastX = event.clientX;
    interaction.lastY = event.clientY;

    if (interaction.mode === "orbit") {
      camera.yaw += dx * bounds.orbitSensitivity;
      camera.pitch = clamp(camera.pitch + dy * bounds.orbitSensitivity, bounds.pitchMin, bounds.pitchMax);
      return;
    }

    camera.panX += dx * bounds.panSensitivity;
    camera.panY += dy * bounds.panSensitivity;
  });

  const clearInteraction = (event) => {
    if (interaction.pointerId !== event.pointerId) return;
    interaction.pointerId = null;
    interaction.mode = null;
  };

  canvas.addEventListener("pointerup", clearInteraction);
  canvas.addEventListener("pointercancel", clearInteraction);

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    const zoomFactor = Math.exp(-event.deltaY * 0.0015);
    camera.zoom = clamp(camera.zoom * zoomFactor, bounds.zoomMin, bounds.zoomMax);
  }, { passive: false });
}
