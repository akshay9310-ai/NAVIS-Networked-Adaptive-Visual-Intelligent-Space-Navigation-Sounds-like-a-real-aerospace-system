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
    // Sanitize command payload to ensure only plain serializable values are sent
    const sanitized: Record<string, any> = {};
    for (const [key, val] of Object.entries(cmd)) {
      if (
        val === null ||
        typeof val === 'number' ||
        typeof val === 'string' ||
        typeof val === 'boolean'
      ) {
        sanitized[key] = val;
      } else if (Array.isArray(val)) {
        sanitized[key] = val.filter(
          (v) => v === null || ['number', 'string', 'boolean'].includes(typeof v)
        );
      } else if (typeof val === 'object' && val !== null && val.constructor === Object) {
        sanitized[key] = val;
      }
    }

    if (wsRef.current && wsRef.current.readyState === WebSocket.OPEN) {
      wsRef.current.send(JSON.stringify(sanitized));
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
      if (actionMap[sanitized.action]) {
        fetch(`${apiBase}${actionMap[sanitized.action]}`, { method: 'POST' });
      }
    }
  }, [apiBase]);

  const startMission = useCallback(() => sendCommand({ action: 'start' }), [sendCommand]);
  const pauseMission = useCallback(() => sendCommand({ action: 'pause' }), [sendCommand]);
  const resetMission = useCallback((seed?: number) => {
    const cleanSeed = typeof seed === 'number' ? seed : undefined;
    sendCommand({ action: 'reset', ...(cleanSeed !== undefined ? { seed: cleanSeed } : {}) });
  }, [sendCommand]);

  const setSpeed = useCallback((multiplier: number) => {
    const cleanMultiplier = typeof multiplier === 'number' ? multiplier : 1.0;
    sendCommand({ action: 'speed', multiplier: cleanMultiplier });
    fetch(`${apiBase}/api/simulation/speed`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ multiplier: cleanMultiplier }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const setMode = useCallback((mode: 'MODE_A' | 'MODE_B' | 'MODE_C') => {
    const cleanMode = typeof mode === 'string' ? mode : 'MODE_C';
    sendCommand({ action: 'mode', mode: cleanMode });
    fetch(`${apiBase}/api/simulation/mode`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ mode: cleanMode }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const simulateOutage = useCallback(() => sendCommand({ action: 'outage' }), [sendCommand]);
  const restoreSatellite = useCallback(() => sendCommand({ action: 'restore' }), [sendCommand]);

  const injectHazard = useCallback((x: number, y: number, radius = 28.0, type = 'ROCK_FIELD') => {
    const cleanX = typeof x === 'number' ? x : 200.0;
    const cleanY = typeof y === 'number' ? y : 200.0;
    const cleanRadius = typeof radius === 'number' ? radius : 28.0;
    const cleanType = typeof type === 'string' ? type : 'ROCK_FIELD';

    sendCommand({ action: 'inject_hazard', x: cleanX, y: cleanY, radius: cleanRadius, type: cleanType });
    fetch(`${apiBase}/api/hazard/inject`, {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ x: cleanX, y: cleanY, radius: cleanRadius, hazard_type: cleanType }),
    }).catch(() => {});
  }, [sendCommand, apiBase]);

  const runDemo = useCallback(() => sendCommand({ action: 'demo' }), [sendCommand]);

  const runBenchmark = useCallback(async (seedInput?: number) => {
    const seed = typeof seedInput === 'number' ? seedInput : 42;
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
