import React from "react";
import { BatteryCharging, Compass, Gauge, ShieldAlert } from "lucide-react";
import { TelemetryState } from "../types";

interface TelemetryGaugesProps {
  telemetry: TelemetryState | null;
}

export const TelemetryGauges: React.FC<TelemetryGaugesProps> = ({ telemetry }) => {
  if (!telemetry) {
    return (
      <div className="panel">
        <p className="hint">Connecting to RoverSAR WebSocket `/ws/rover`...</p>
      </div>
    );
  }

  const maxRpm = 180;
  const leftPct = Math.min(100, (Math.abs(telemetry.rpm.left) / maxRpm) * 100);
  const rightPct = Math.min(100, (Math.abs(telemetry.rpm.right) / maxRpm) * 100);
  const wdPct = Math.min(
    100,
    (telemetry.watchdog_ms / telemetry.watchdog_timeout_ms) * 100
  );
  const yawDeg = ((telemetry.imu.yaw * 180) / Math.PI).toFixed(1);

  return (
    <div className="panel gauges-panel">
      <div className="panel-header">
        <h3>Telemetry &amp; Power Diagnostics</h3>
        <span className="mono">
          POSE ({telemetry.pose.x.toFixed(2)}m, {telemetry.pose.y.toFixed(2)}m)
        </span>
      </div>

      <div className="gauge-grid">
        <div className="metric-card">
          <div className="metric-title">
            <Gauge size={15} /> Left Motor RPM
          </div>
          <div className="metric-val">{telemetry.rpm.left.toFixed(1)} RPM</div>
          <div className="bar-track">
            <div className="bar-fill cyan" style={{ width: `${leftPct}%` }} />
          </div>
        </div>

        <div className="metric-card">
          <div className="metric-title">
            <Gauge size={15} /> Right Motor RPM
          </div>
          <div className="metric-val">{telemetry.rpm.right.toFixed(1)} RPM</div>
          <div className="bar-track">
            <div className="bar-fill cyan" style={{ width: `${rightPct}%` }} />
          </div>
        </div>

        <div className="metric-card">
          <div className="metric-title">
            <BatteryCharging size={15} /> 2S Li-Ion Pack
          </div>
          <div className="metric-val">
            {telemetry.battery.v.toFixed(2)}V ({telemetry.battery.soc.toFixed(0)}%)
          </div>
          <div className="bar-track">
            <div
              className={`bar-fill ${telemetry.battery.low_warn ? "red" : "green"}`}
              style={{ width: `${telemetry.battery.soc}%` }}
            />
          </div>
        </div>

        <div className="metric-card">
          <div className="metric-title">
            <ShieldAlert size={15} /> Watchdog (500ms)
          </div>
          <div className="metric-val">{telemetry.watchdog_ms} ms</div>
          <div className="bar-track">
            <div
              className={`bar-fill ${wdPct >= 80 ? "red" : wdPct >= 50 ? "amber" : "green"}`}
              style={{ width: `${wdPct}%` }}
            />
          </div>
        </div>
      </div>

      <div className="imu-strip">
        <span>
          <Compass size={14} /> Yaw: <strong>{yawDeg}°</strong>
        </span>
        <span>
          V: <strong>{telemetry.pose.v.toFixed(2)} m/s</strong>
        </span>
        <span>
          ω: <strong>{telemetry.pose.w.toFixed(2)} rad/s</strong>
        </span>
        <span>
          ToF Center: <strong>{(telemetry.tof_m[2] ?? 2.0).toFixed(2)} m</strong>
        </span>
      </div>
    </div>
  );
};
