import React from "react";
import { Activity, Wifi, WifiOff } from "lucide-react";
import { CameraHudPanel } from "./components/CameraHudPanel";
import { OccupancyMapCanvas } from "./components/OccupancyMapCanvas";
import { SafetyControls } from "./components/SafetyControls";
import { TelemetryGauges } from "./components/TelemetryGauges";
import { VirtualJoystick } from "./components/VirtualJoystick";
import { useRoverSocket } from "./hooks/useRoverSocket";

export const App: React.FC = () => {
  const { telemetry, connected, commLossSim, sendCommand } = useRoverSocket();
  const driveDisabled =
    !connected ||
    telemetry?.mode === "ESTOP" ||
    telemetry?.fault === "ESTOP_SOFTWARE" ||
    telemetry?.fault === "ESTOP_HARDWARE";

  return (
    <div className="hud-shell">
      <header className="topbar">
        <div className="brand">
          <Activity size={20} className="brand-icon" />
          <div>
            <h1>ROVERSAR // AUTONOMOUS SAR GROUND STATION</h1>
            <span className="subtitle">
              ESP32 Diff-Drive • 5x VL53L0X ToF SLAM • 500ms Safety Watchdog
            </span>
          </div>
        </div>
        <div className="topbar-status">
          <span className={`pill ${connected ? "pill-ok" : "pill-err"}`}>
            {connected ? <Wifi size={14} /> : <WifiOff size={14} />}
            {connected ? "WS CONNECTED" : "RECONNECTING"}
          </span>
          <span className="pill pill-mode">
            MODE: {telemetry?.mode ?? "INIT"}
          </span>
        </div>
      </header>

      <main className="hud-grid">
        <section className="col-map">
          <OccupancyMapCanvas
            telemetry={telemetry}
            onSetWaypoint={(x, y) => sendCommand({ type: "waypoint", x, y })}
          />
        </section>

        <section className="col-side">
          <SafetyControls
            telemetry={telemetry}
            commLossSim={commLossSim}
            onCommand={sendCommand}
          />
          <CameraHudPanel telemetry={telemetry} />
          <TelemetryGauges telemetry={telemetry} />
          <VirtualJoystick
            disabled={driveDisabled}
            onDrive={(linear, angular) =>
              sendCommand({ type: "cmd_vel", linear, angular })
            }
          />
        </section>
      </main>
    </div>
  );
};

export default App;
