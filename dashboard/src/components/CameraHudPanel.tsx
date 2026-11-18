import React, { useEffect, useState } from "react";
import { Camera } from "lucide-react";
import { TelemetryState } from "../types";

interface CameraHudPanelProps {
  telemetry: TelemetryState | null;
}

export const CameraHudPanel: React.FC<CameraHudPanelProps> = ({ telemetry }) => {
  const [frameSrc, setFrameSrc] = useState("/api/camera/frame.jpg?t=0");

  useEffect(() => {
    const timer = window.setInterval(() => {
      setFrameSrc(`/api/camera/frame.jpg?t=${Date.now()}`);
    }, 140);
    return () => window.clearInterval(timer);
  }, []);

  const minFront = telemetry
    ? Math.min(
        telemetry.tof_m[1] ?? 2.0,
        telemetry.tof_m[2] ?? 2.0,
        telemetry.tof_m[3] ?? 2.0
      )
    : 2.0;

  return (
    <div className="panel camera-panel">
      <div className="panel-header">
        <h3>
          <Camera size={16} /> ESP32-CAM Tactical FPV
        </h3>
        <span className={`badge ${minFront < 0.35 ? "badge-red" : "badge-green"}`}>
          CLEARANCE {minFront.toFixed(2)} m
        </span>
      </div>
      <div className="camera-frame-box">
        <img
          src={frameSrc}
          alt="RoverSAR FPV ESP32-CAM feed"
          className="camera-img"
        />
      </div>
    </div>
  );
};
