import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { BackendEvent, emptyState, PalletizerState } from "../lib/types";

const WS_URL = import.meta.env.VITE_PALLETIZER_WS_URL ?? "ws://127.0.0.1:8765";

export function usePalletizerSocket() {
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectRef = useRef<number | null>(null);
  const [connected, setConnected] = useState(false);
  const [state, setState] = useState<PalletizerState>(emptyState);
  const [events, setEvents] = useState<BackendEvent[]>([]);
  const [lastFeedback, setLastFeedback] = useState<Record<string, unknown> | null>(null);
  const [lastResult, setLastResult] = useState<Record<string, unknown> | null>(null);

  const pushEvent = useCallback((event: BackendEvent) => {
    setEvents((current) => [event, ...current].slice(0, 80));
    if (event.state) setState(event.state);
    if (event.type === "action_feedback") setLastFeedback({ action: event.action, ...event.feedback });
    if (event.type === "action_result") setLastResult({ action: event.action, ...event.result });
  }, []);

  const connect = useCallback(() => {
    if (socketRef.current && socketRef.current.readyState <= WebSocket.OPEN) return;
    const socket = new WebSocket(WS_URL);
    socketRef.current = socket;

    socket.onopen = () => {
      setConnected(true);
      socket.send(JSON.stringify({ type: "refresh" }));
    };

    socket.onmessage = (message) => {
      try {
        pushEvent(JSON.parse(message.data));
      } catch (error) {
        pushEvent({ type: "parse_error", message: String(error) });
      }
    };

    socket.onclose = () => {
      setConnected(false);
      reconnectRef.current = window.setTimeout(connect, 1200);
    };

    socket.onerror = () => {
      socket.close();
    };
  }, [pushEvent]);

  useEffect(() => {
    connect();
    return () => {
      if (reconnectRef.current) window.clearTimeout(reconnectRef.current);
      socketRef.current?.close();
    };
  }, [connect]);

  const send = useCallback((payload: Record<string, unknown>) => {
    const socket = socketRef.current;
    if (!socket || socket.readyState !== WebSocket.OPEN) {
      pushEvent({ type: "local_error", message: "WebSocket no conectado" });
      return;
    }
    socket.send(JSON.stringify(payload));
  }, [pushEvent]);

  return useMemo(() => ({
    connected,
    state,
    events,
    lastFeedback,
    lastResult,
    send
  }), [connected, events, lastFeedback, lastResult, send, state]);
}
