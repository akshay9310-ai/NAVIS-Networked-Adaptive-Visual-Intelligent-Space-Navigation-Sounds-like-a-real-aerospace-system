import { useState, useEffect, useRef, useCallback } from 'react';
import { SimulationFullState, BenchmarkComparisonResult } from '../types/simulation';

export function useSimulation() {
  const [state, setState] = useState<SimulationFullState | null>(null);
  const [connected, setConnected] = useState<boolean>(false);
  const [benchmarkResult, setBenchmarkResult] = useState<BenchmarkComparisonResult | null>(null);
  const [isBenchmarking, setIsBenchmarking] = useState<boolean>(false);
  const wsRef = useRef<WebSocket | null>(null);
  const reconnectTimeoutRef = useRef<number | null>(null);

  // Dynamic API & WS Base Resolution
  const apiBase = typeof window !== 'undefined' && window.location.port === '5173' ? 'http://127.0.0.1:8000' : '';

  // Connect to WebSocket
  const connectWs = useCallback(() => {
    const protocol = window.location.protocol === 'https:' ? 'wss:' : 'ws:';
    const wsUrl =
      window.location.port === '5173'
        ? `${protocol}//${window.location.hostname}:8000/ws/simulation`
        : `${protocol}//${window.location.host}/ws/simulation`;

    try {
      const ws = new WebSocket(wsUrl);
      wsRef.current = ws;

      ws.onopen = () => {
        setConnected(true);
      };

      ws.onmessage = (event) => {
        try {
          const data: SimulationFullState = JSON.parse(event.data);
          setState(data);
        } catch (err) {
          console.error('Error parsing simulation WebSocket message:', err);
        }
      };

      ws.onclose = () => {
        setConnected(false);
        // Attempt reconnect after 1.5s
        reconnectTimeoutRef.current = window.setTimeout(() => {
          connectWs();
        }, 1500);
      };

      ws.onerror = () => {
        setConnected(false);
        ws.close();
      };
    } catch (e) {
      console.error('Failed to create WebSocket:', e);
      reconnectTimeoutRef.current = window.setTimeout(connectWs, 2000);
    }
  }, []);

  useEffect(() => {
    connectWs();
    return () => {
      if (reconnectTimeoutRef.current) clearTimeout(reconnectTimeoutRef.current);
      if (wsRef.current) wsRef.current.close();
    };
  }, [connectWs]);

  // Fallback REST polling if WS is disconnected
  useEffect(() => {
    if (connected) return;
    const interval = setInterval(async () => {
      try {
        const res = await fetch(`${apiBase}/api/simulation/state`);
        if (res.ok) {
          const data = await res.json();
          setState(data);
        }
      } catch {
        // server might still be booting
      }
    }, 1000);
    return () => clearInterval(interval);
  }, [connected, apiBase]);

  // Send Command Helper
  const sendCommand = useCallback((cmd: Record<string, any>) => {
    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(cmd));
    } else {
      // Fallback REST POST
      const actionMap: Record<string, string> = {
        start: '/api/simulation/start',
        pause: '/api/simulation/pause',
        reset: '/api/simulation/reset',
        outage: '/api/satellite/outage',
        restore: '/api/satellite/restore',
        demo: '/api/demo/run',
      };
      if (actionMap[cmd.action]) {
        fetch(`${apiBase}${actionMap[cmd.action]}`, { method: 'POST' });
      }
    }
  }, [apiBase]);

  const startMission = useCallback(() => sendCommand({ action: 'start' }), [sendCommand]);
  const pauseMission = useCallback(() => sendCommand({ action: 'pause' }), [sendCommand]);
  const resetMission = useCallback((seed?: number) => sendCommand({ action: 'reset', seed }), [sendCommand]);
  const setSpeed = useCallback((multiplier: number) => {
    sendCommand({ action: 'speed', multiplier });
    fetch(`${apiBase}/api/simulation/speed`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ multiplier }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const setMode = useCallback((mode: 'MODE_A' | 'MODE_B' | 'MODE_C') => {
    sendCommand({ action: 'mode', mode });
    fetch(`${apiBase}/api/simulation/mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const simulateOutage = useCallback(() => sendCommand({ action: 'outage' }), [sendCommand]);
  const restoreSatellite = useCallback(() => sendCommand({ action: 'restore' }), [sendCommand]);

  const injectHazard = useCallback((x: number, y: number, radius = 28.0, type = 'ROCK_FIELD') => {
    sendCommand({ action: 'inject_hazard', x, y, radius, type });
    fetch(`${apiBase}/api/hazard/inject`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x, y, radius, hazard_type: type }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const runDemo = useCallback(() => sendCommand({ action: 'demo' }), [sendCommand]);

  const runBenchmark = useCallback(async (seed = 42) => {
    setIsBenchmarking(true);
    try {
      const res = await fetch(`${apiBase}/api/benchmark/run`, {
        method: 'POST',
        headers: { 'Content-Type': 'application/json' },
        body: JSON.stringify({ seed }),
      });
      if (res.ok) {
        const data: BenchmarkComparisonResult = await res.json();
        setBenchmarkResult(data);
      }
    } catch (e) {
      console.error('Failed to run benchmark:', e);
    } finally {
      setIsBenchmarking(false);
    }
  }, [apiBase]);

  return {
    state,
    connected,
    benchmarkResult,
    isBenchmarking,
    startMission,
    pauseMission,
    resetMission,
    setSpeed,
    setMode,
    simulateOutage,
    restoreSatellite,
    injectHazard,
    runDemo,
    runBenchmark,
    clearBenchmark: () => setBenchmarkResult(null),
  };
}
