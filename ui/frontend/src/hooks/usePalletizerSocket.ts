import { useCallback, useEffect, useMemo, useRef, useState } from "react";
import { BackendEvent, emptyState, PalletizerState } from "../lib/types";

const WS_URL = import.meta.env.VITE_PALLETIZER_WS_URL ?? "ws://127.0.0.1:8765";
const EVENT_LOG_IGNORED_TYPES = new Set(["state", "backend_status", "hello"]);

export function usePalletizerSocket() {
  const socketRef = useRef<WebSocket | null>(null);
  const reconnectRef = useRef<number | null>(null);
  const shouldReconnectRef = useRef(true);
  const [connected, setConnected] = useState(false);
  const [state, setState] = useState<PalletizerState>(emptyState);
  const [events, setEvents] = useState<BackendEvent[]>([]);
  const [lastFeedback, setLastFeedback] = useState<Record<string, unknown> | null>(null);
  const [lastResult, setLastResult] = useState<Record<string, unknown> | null>(null);
  const [lastPing, setLastPing] = useState<Record<string, unknown> | null>(null);

  const pushEvent = useCallback((event: BackendEvent) => {
    if (event.state) setState(event.state);
    if (event.type === "ping_result" && event.ping) setLastPing(event.ping);
    if (event.type === "action_feedback") setLastFeedback({ action: event.action, ...event.feedback });
    if (event.type === "action_result") setLastResult({ action: event.action, ...event.result });
    if (!EVENT_LOG_IGNORED_TYPES.has(event.type)) {
      setEvents((current) => [event, ...current].slice(0, 80));
    }
  }, []);

  const connect = useCallback(() => {
    if (socketRef.current && socketRef.current.readyState <= WebSocket.OPEN) return;
    const socket = new WebSocket(WS_URL);
    socketRef.current = socket;

    socket.onopen = () => {
      if (socketRef.current !== socket) return;
      setConnected(true);
      socket.send(JSON.stringify({ type: "refresh" }));
    };

    socket.onmessage = (message) => {
      if (socketRef.current !== socket) return;
      try {
        pushEvent(JSON.parse(message.data));
      } catch (error) {
        pushEvent({ type: "parse_error", message: String(error) });
      }
    };

    socket.onclose = () => {
      if (socketRef.current === socket) socketRef.current = null;
      setConnected(false);
      if (!shouldReconnectRef.current) return;
      if (reconnectRef.current) window.clearTimeout(reconnectRef.current);
      reconnectRef.current = window.setTimeout(connect, 1200);
    };

    socket.onerror = () => {
      if (socketRef.current === socket) socket.close();
    };
  }, [pushEvent]);

  useEffect(() => {
    shouldReconnectRef.current = true;
    connect();
    return () => {
      shouldReconnectRef.current = false;
      if (reconnectRef.current) window.clearTimeout(reconnectRef.current);
      reconnectRef.current = null;
      const socket = socketRef.current;
      socketRef.current = null;
      socket?.close();
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
    lastPing,
    send
  }), [connected, events, lastFeedback, lastPing, lastResult, send, state]);
}
