import React from "react";
import { OctagonAlert, RadioTower, RotateCcw, ShieldCheck } from "lucide-react";
import { DriveMode, OutboundCommand, TelemetryState } from "../types";

interface SafetyControlsProps {
  telemetry: TelemetryState | null;
  commLossSim: boolean;
  onCommand: (cmd: OutboundCommand) => void;
}

export const SafetyControls: React.FC<SafetyControlsProps> = ({
  telemetry,
  commLossSim,
  onCommand,
}) => {
  const mode = telemetry?.mode ?? "IDLE";
  const fault = telemetry?.fault ?? "NONE";
  const isEstop = mode === "ESTOP" || fault === "ESTOP_SOFTWARE" || fault === "ESTOP_HARDWARE";

  return (
    <div className="panel safety-panel">
      <div className="panel-header">
        <h3>Safety &amp; Mission Control</h3>
        <span className={`badge ${fault === "NONE" ? "badge-green" : "badge-red"}`}>
          FAULT: {fault}
        </span>
      </div>

      <div className="safety-actions">
        <button
          type="button"
          className="estop-btn"
          onClick={() => onCommand({ type: "estop" })}
        >
          <OctagonAlert size={20} /> EMERGENCY E-STOP
        </button>

        {isEstop && (
          <button
            type="button"
            className="btn btn-clear"
            onClick={() => onCommand({ type: "clear_estop" })}
          >
            <ShieldCheck size={16} /> Clear E-STOP &amp; Re-Arm
          </button>
        )}

        <button
          type="button"
          className={`btn ${commLossSim ? "btn-warn-active" : "btn-warn"}`}
          onClick={() => onCommand({ type: "comm_loss", active: !commLossSim })}
        >
          <RadioTower size={16} />
          {commLossSim
            ? "Restore Heartbeat Link"
            : "Simulate Comm Loss (500ms Test)"}
        </button>

        <div className="mode-row">
          {(["IDLE", "MANUAL", "AUTONOMOUS"] as DriveMode[]).map((m) => (
            <button
              key={m}
              type="button"
              disabled={isEstop}
              className={`btn-mode ${mode === m ? "active" : ""}`}
              onClick={() => onCommand({ type: "mode", mode: m })}
            >
              {m}
            </button>
          ))}
        </div>

        <button
          type="button"
          className="btn btn-secondary"
          onClick={() => onCommand({ type: "reset" })}
        >
          <RotateCcw size={15} /> Reset Simulation &amp; Map
        </button>
      </div>
    </div>
  );
};
