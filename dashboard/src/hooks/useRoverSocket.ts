import { useCallback, useEffect, useRef, useState } from "react";
import { OutboundCommand, TelemetryState } from "../types";

export function useRoverSocket() {
  const [telemetry, setTelemetry] = useState<TelemetryState | null>(null);
  const [connected, setConnected] = useState(false);
  const [commLossSim, setCommLossSim] = useState(false);
  const wsRef = useRef<WebSocket | null>(null);
  const commLossRef = useRef(false);

  useEffect(() => {
    commLossRef.current = commLossSim;
  }, [commLossSim]);

  useEffect(() => {
    let disposed = false;
    let reconnectTimer: number | undefined;

    const connect = () => {
      if (disposed) return;
      const proto = window.location.protocol === "https:" ? "wss:" : "ws:";
      const wsUrl = `${proto}//${window.location.host}/ws/rover`;
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
      };

      ws.onmessage = (evt) => {
        try {
          const parsed = JSON.parse(evt.data) as TelemetryState;
          setTelemetry(parsed);
          setCommLossSim(parsed.comm_loss_injected);
        } catch {
          // Ignore malformed frame
        }
      };

      ws.onclose = () => {
        setConnected(false);
        wsRef.current = null;
        if (!disposed) {
          reconnectTimer = window.setTimeout(connect, 1000);
        }
      };
    };

    connect();

    // 200ms heartbeat feed unless comm-loss simulation is intentionally active
    const hbTimer = window.setInterval(() => {
      if (
        wsRef.current &&
        wsRef.current.readyState === WebSocket.OPEN &&
        !commLossRef.current
      ) {
        wsRef.current.send(JSON.stringify({ type: "heartbeat" }));
      }
    }, 200);

    return () => {
      disposed = true;
      window.clearInterval(hbTimer);
      if (reconnectTimer) window.clearTimeout(reconnectTimer);
      wsRef.current?.close();
    };
  }, []);

  const sendCommand = useCallback((cmd: OutboundCommand) => {
    if (cmd.type === "comm_loss") {
      setCommLossSim(cmd.active);
      commLossRef.current = cmd.active;
    }
    if (cmd.type === "clear_estop" || cmd.type === "reset") {
      setCommLossSim(false);
      commLossRef.current = false;
    }
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
      return;
    }
    // HTTP fallback if WS temporarily disconnected
    if (cmd.type === "estop") {
      void fetch("/api/estop", { method: "POST" });
    } else if (cmd.type === "clear_estop") {
      void fetch("/api/clear-estop", { method: "POST" });
    } else if (cmd.type === "comm_loss") {
      void fetch("/api/comm-loss", {
        method: "POST",
        headers: { "Content-Type": "application/json" },
        body: JSON.stringify({ active: cmd.active }),
      });
    }
  }, []);

  return { telemetry, connected, commLossSim, sendCommand };
}
