import React, { useEffect, useRef } from "react";
import { TelemetryState } from "../types";

interface OccupancyMapCanvasProps {
  telemetry: TelemetryState | null;
  onSetWaypoint: (x: number, y: number) => void;
}

export const OccupancyMapCanvas: React.FC<OccupancyMapCanvasProps> = ({
  telemetry,
  onSetWaypoint,
}) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !telemetry) return;
    const ctx = canvas.getContext("2d");
    if (!ctx) return;

    const size = canvas.width;
    const worldMin = -5.0;
    const worldSpan = 10.0;
    const scale = size / worldSpan;

    const toPx = (wx: number, wy: number): [number, number] => [
      (wx - worldMin) * scale,
      size - (wy - worldMin) * scale,
    ];

    // 1. Background
    ctx.fillStyle = "#080d18";
    ctx.fillRect(0, 0, size, size);

    // 2. Live 2D Occupancy Grid
    if (telemetry.grid) {
      const { width, height, res, origin_x, origin_y, cells } = telemetry.grid;
      const cellPx = res * scale;
      for (let gy = 0; gy < height; gy++) {
        for (let gx = 0; gx < width; gx++) {
          const val = cells[gy * width + gx];
          if (val === -1) continue;
          const wx = origin_x + gx * res;
          const wy = origin_y + (gy + 1) * res;
          const [px, py] = toPx(wx, wy);
          ctx.fillStyle = val === 1 ? "rgba(249, 115, 22, 0.72)" : "rgba(30, 58, 95, 0.55)";
          ctx.fillRect(px, py, Math.ceil(cellPx), Math.ceil(cellPx));
        }
      }
    }

    // 3. Subtle 1m coordinate grid
    ctx.strokeStyle = "rgba(148, 163, 184, 0.12)";
    ctx.lineWidth = 1;
    for (let m = -4; m <= 4; m++) {
      const [gx] = toPx(m, 0);
      const [, gy] = toPx(0, m);
      ctx.beginPath();
      ctx.moveTo(gx, 0);
      ctx.lineTo(gx, size);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(0, gy);
      ctx.lineTo(size, gy);
      ctx.stroke();
    }

    // 4. Indoor SAR structural walls overlay (subtle tactical blueprint lines)
    if (telemetry.world) {
      ctx.strokeStyle = "rgba(148, 163, 184, 0.45)";
      ctx.lineWidth = 2;
      for (const [x1, y1, x2, y2] of telemetry.world.walls) {
        const [px1, py1] = toPx(x1, y1);
        const [px2, py2] = toPx(x2, y2);
        ctx.beginPath();
        ctx.moveTo(px1, py1);
        ctx.lineTo(px2, py2);
        ctx.stroke();
      }
    }

    // 5. Rescue Beacon Targets
    for (const tgt of telemetry.targets) {
      const [tx, ty] = toPx(tgt.x, tgt.y);
      ctx.beginPath();
      ctx.arc(tx, ty, 7, 0, Math.PI * 2);
      ctx.fillStyle = tgt.detected ? "#10b981" : "#f59e0b";
      ctx.fill();
      ctx.strokeStyle = tgt.detected ? "#6ee7b7" : "#fde68a";
      ctx.lineWidth = 2;
      ctx.stroke();
      ctx.fillStyle = "#e2e8f0";
      ctx.font = "10px monospace";
      ctx.fillText(tgt.id, tx + 10, ty + 3);
    }

    // 6. Active Waypoint Target
    if (telemetry.waypoint) {
      const [wx, wy] = toPx(telemetry.waypoint.x, telemetry.waypoint.y);
      ctx.strokeStyle = "#38bdf8";
      ctx.lineWidth = 2;
      ctx.beginPath();
      ctx.arc(wx, wy, 10, 0, Math.PI * 2);
      ctx.stroke();
      ctx.beginPath();
      ctx.moveTo(wx - 14, wy);
      ctx.lineTo(wx + 14, wy);
      ctx.moveTo(wx, wy - 14);
      ctx.lineTo(wx, wy + 14);
      ctx.stroke();
    }

    // 7. VL53L0X ToF Laser Rays from Rover Pose
    const rx = telemetry.pose.x;
    const ry = telemetry.pose.y;
    const rtheta = telemetry.pose.theta;
    const sensorWorldX = rx + 0.105 * Math.cos(rtheta);
    const sensorWorldY = ry + 0.105 * Math.sin(rtheta);
    const [sx, sy] = toPx(sensorWorldX, sensorWorldY);

    telemetry.tof_m.forEach((dist, idx) => {
      const rel = telemetry.tof_angles_rad[idx] ?? 0;
      const beamAng = rtheta + rel;
      const hitX = sensorWorldX + dist * Math.cos(beamAng);
      const hitY = sensorWorldY + dist * Math.sin(beamAng);
      const [hx, hy] = toPx(hitX, hitY);

      const close = dist < 0.55;
      ctx.strokeStyle = close ? "rgba(239, 68, 68, 0.85)" : "rgba(34, 211, 238, 0.72)";
      ctx.lineWidth = close ? 2 : 1.5;
      ctx.beginPath();
      ctx.moveTo(sx, sy);
      ctx.lineTo(hx, hy);
      ctx.stroke();

      if (dist < 1.95) {
        ctx.fillStyle = close ? "#ef4444" : "#22d3ee";
        ctx.beginPath();
        ctx.arc(hx, hy, 3, 0, Math.PI * 2);
        ctx.fill();
      }
    });

    // 8. Rover 240x180mm Chassis Footprint
    const [rpx, rpy] = toPx(rx, ry);
    ctx.save();
    ctx.translate(rpx, rpy);
    ctx.rotate(-rtheta);
    const lenPx = 0.24 * scale;
    const widPx = 0.18 * scale;
    ctx.fillStyle = telemetry.fault !== "NONE" ? "#ef4444" : "#0ea5e9";
    ctx.strokeStyle = "#f8fafc";
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.roundRect(-lenPx / 2, -widPx / 2, lenPx, widPx, 4);
    ctx.fill();
    ctx.stroke();
    // Forward heading arrow
    ctx.fillStyle = "#fde047";
    ctx.beginPath();
    ctx.moveTo(lenPx / 2 + 4, 0);
    ctx.lineTo(lenPx / 2 - 5, -5);
    ctx.lineTo(lenPx / 2 - 5, 5);
    ctx.closePath();
    ctx.fill();
    ctx.restore();
  }, [telemetry]);

  const handleMapClick = (e: React.MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;
    const rect = canvas.getBoundingClientRect();
    const cx = ((e.clientX - rect.left) / rect.width) * canvas.width;
    const cy = ((e.clientY - rect.top) / rect.height) * canvas.height;
    const worldX = -5.0 + (cx / canvas.width) * 10.0;
    const worldY = -5.0 + ((canvas.height - cy) / canvas.height) * 10.0;
    onSetWaypoint(Number(worldX.toFixed(2)), Number(worldY.toFixed(2)));
  };

  return (
    <div className="panel map-panel">
      <div className="panel-header">
        <div>
          <h2>2D Occupancy SLAM &amp; VL53L0X LiDAR</h2>
          <span className="hint">Click or tap anywhere inside arena to dispatch waypoint</span>
        </div>
        {telemetry && (
          <div className="map-badges">
            <span className="badge badge-cyan">
              Explored: {telemetry.map_stats.explored_pct}%
            </span>
            {telemetry.avoidance && (
              <span className="badge badge-amber">VFH AVOIDANCE</span>
            )}
          </div>
        )}
      </div>
      <div className="canvas-wrap">
        <canvas
          ref={canvasRef}
          width={560}
          height={560}
          onClick={handleMapClick}
          className="map-canvas"
          aria-label="2D Occupancy Map and ToF Raycast Canvas"
        />
      </div>
    </div>
  );
};
