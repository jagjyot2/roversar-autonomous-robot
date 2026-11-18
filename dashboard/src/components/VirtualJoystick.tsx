import React, { useCallback, useEffect, useRef, useState } from "react";

interface VirtualJoystickProps {
  disabled: boolean;
  onDrive: (linear: number, angular: number) => void;
}

const MAX_LINEAR = 0.52;
const MAX_ANGULAR = 2.4;

export const VirtualJoystick: React.FC<VirtualJoystickProps> = ({
  disabled,
  onDrive,
}) => {
  const padRef = useRef<HTMLDivElement | null>(null);
  const [knob, setKnob] = useState({ x: 0, y: 0 });
  const [dragging, setDragging] = useState(false);
  const keysPressed = useRef<Set<string>>(new Set());

  const emitFromNormalized = useCallback(
    (nx: number, ny: number) => {
      if (disabled) return;
      // Forward is negative screen Y; left turn (CCW +w) is negative screen X
      const linear = Number((-ny * MAX_LINEAR).toFixed(3));
      const angular = Number((-nx * MAX_ANGULAR).toFixed(3));
      onDrive(linear, angular);
    },
    [disabled, onDrive]
  );

  // Continuous keyboard WASD + Arrow keys driving
  useEffect(() => {
    const computeKeyDrive = () => {
      if (disabled) return;
      const keys = keysPressed.current;
      let ny = 0;
      let nx = 0;
      if (keys.has("w") || keys.has("arrowup")) ny -= 1;
      if (keys.has("s") || keys.has("arrowdown")) ny += 1;
      if (keys.has("a") || keys.has("arrowleft")) nx -= 1;
      if (keys.has("d") || keys.has("arrowright")) nx += 1;
      setKnob({ x: nx * 42, y: ny * 42 });
      emitFromNormalized(nx, ny);
    };

    const onKeyDown = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (["w", "a", "s", "d", "arrowup", "arrowdown", "arrowleft", "arrowright"].includes(k)) {
        e.preventDefault();
        if (!keysPressed.current.has(k)) {
          keysPressed.current.add(k);
          computeKeyDrive();
        }
      }
    };

    const onKeyUp = (e: KeyboardEvent) => {
      const k = e.key.toLowerCase();
      if (keysPressed.current.has(k)) {
        keysPressed.current.delete(k);
        computeKeyDrive();
      }
    };

    window.addEventListener("keydown", onKeyDown);
    window.addEventListener("keyup", onKeyUp);
    return () => {
      window.removeEventListener("keydown", onKeyDown);
      window.removeEventListener("keyup", onKeyUp);
    };
  }, [disabled, emitFromNormalized]);

  const updatePointer = (clientX: number, clientY: number) => {
    const pad = padRef.current;
    if (!pad || disabled) return;
    const rect = pad.getBoundingClientRect();
    const radius = rect.width / 2 - 18;
    const dx = clientX - (rect.left + rect.width / 2);
    const dy = clientY - (rect.top + rect.height / 2);
    const dist = Math.hypot(dx, dy);
    const clampedDist = Math.min(dist, radius);
    const ang = Math.atan2(dy, dx);
    const cx = clampedDist * Math.cos(ang);
    const cy = clampedDist * Math.sin(ang);
    setKnob({ x: cx, y: cy });
    emitFromNormalized(cx / radius, cy / radius);
  };

  const releaseStick = () => {
    setDragging(false);
    setKnob({ x: 0, y: 0 });
    if (!disabled) {
      onDrive(0, 0);
    }
  };

  return (
    <div className="panel joystick-panel">
      <div className="panel-header">
        <h3>Manual Drive (Touch / WASD)</h3>
        <span className="hint">50 Hz PID Loop</span>
      </div>
      <div className="joystick-body">
        <div
          ref={padRef}
          className={`joystick-base ${disabled ? "disabled" : ""}`}
          onPointerDown={(e) => {
            if (disabled) return;
            e.currentTarget.setPointerCapture(e.pointerId);
            setDragging(true);
            updatePointer(e.clientX, e.clientY);
          }}
          onPointerMove={(e) => {
            if (dragging) updatePointer(e.clientX, e.clientY);
          }}
          onPointerUp={releaseStick}
          onPointerCancel={releaseStick}
        >
          <div className="joystick-crosshair-h" />
          <div className="joystick-crosshair-v" />
          <div
            className="joystick-knob"
            style={{ transform: `translate(${knob.x}px, ${knob.y}px)` }}
          />
        </div>
        <div className="wasd-grid" aria-label="WASD virtual keypad">
          <div />
          <button
            type="button"
            disabled={disabled}
            onPointerDown={() => emitFromNormalized(0, -1)}
            onPointerUp={() => emitFromNormalized(0, 0)}
          >
            W
          </button>
          <div />
          <button
            type="button"
            disabled={disabled}
            onPointerDown={() => emitFromNormalized(-1, 0)}
            onPointerUp={() => emitFromNormalized(0, 0)}
          >
            A
          </button>
          <button
            type="button"
            disabled={disabled}
            onPointerDown={() => emitFromNormalized(0, 1)}
            onPointerUp={() => emitFromNormalized(0, 0)}
          >
            S
          </button>
          <button
            type="button"
            disabled={disabled}
            onPointerDown={() => emitFromNormalized(1, 0)}
            onPointerUp={() => emitFromNormalized(0, 0)}
          >
            D
          </button>
        </div>
      </div>
    </div>
  );
};
