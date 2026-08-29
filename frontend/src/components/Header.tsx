import React, { useState, useEffect } from 'react';
import { Satellite, Radio, ShieldCheck, Activity, Compass, Cpu, AlertTriangle } from 'lucide-react';
import { SimulationFullState } from '../types/simulation';

interface HeaderProps {
  state: SimulationFullState | null;
  connected: boolean;
  onOpenBenchmark: () => void;
}

export const Header: React.FC<HeaderProps> = ({ state, connected, onOpenBenchmark }) => {
  const [utcTime, setUtcTime] = useState<string>('');

  useEffect(() => {
    const updateTime = () => {
      const now = new Date();
      setUtcTime(now.toUTCString().slice(17, 25) + ' UTC');
    };
    updateTime();
    const interval = setInterval(updateTime, 1000);
    return () => clearInterval(interval);
  }, []);

  const metMinutes = state ? Math.floor(state.sim_time / 60) : 0;
  const metSeconds = state ? Math.floor(state.sim_time % 60) : 0;
  const metString = `MET +${String(metMinutes).padStart(2, '0')}:${String(metSeconds).padStart(2, '0')}`;

  const modeBadgeColor = {
    MODE_A: 'bg-amber-500/10 text-amber-400 border-amber-500/30',
    MODE_B: 'bg-blue-500/10 text-blue-400 border-blue-500/30',
    MODE_C: 'bg-neon-cyan/10 text-neon-cyan border-neon-cyan/30 shadow-neon-cyan/20',
  }[state?.active_mode || 'MODE_C'];

  return (
    <header className="w-full bg-space-900/90 border-b border-slate-800/80 px-4 py-2.5 flex flex-wrap items-center justify-between gap-4 select-none backdrop-blur-md">
      {/* Brand & Subtitle */}
      <div className="flex items-center gap-3">
        <div className="relative flex items-center justify-center w-10 h-10 rounded-xl bg-gradient-to-br from-cyan-500/20 via-blue-600/20 to-purple-600/20 border border-neon-cyan/40 shadow-neon-cyan">
          <Satellite className="w-5 h-5 text-neon-cyan animate-pulse-slow" />
          <div className="absolute inset-0 rounded-xl border border-neon-cyan/20 animate-ping opacity-25" />
        </div>
        <div>
          <div className="flex items-center gap-2">
            <h1 className="font-mono text-lg font-black tracking-wider text-transparent bg-clip-text bg-gradient-to-r from-neon-cyan via-white to-blue-400">
              NAVIS
            </h1>
            <span className="text-[10px] font-mono uppercase px-1.5 py-0.5 rounded bg-slate-800 text-slate-300 border border-slate-700">
              v2.4 SP
            </span>
          </div>
          <p className="text-[11px] text-slate-400 font-medium tracking-tight">
            Networked Adaptive Visual & Intelligent Space Navigation • <span className="text-neon-cyan font-mono text-[10px]">Adaptive Intelligence Beyond Earth</span>
          </p>
        </div>
      </div>

      {/* Center Mission Status Banners */}
      <div className="flex items-center gap-3">
        {/* Active Mode */}
        <div className={`badge-neon border px-3 py-1 ${modeBadgeColor}`}>
          <Cpu className="w-3.5 h-3.5" />
          <span className="font-bold text-xs">{state?.mode_meta.tag || 'ADAPTIVE AI'}</span>
        </div>

        {/* Fallback Alarm Banner */}
        {state?.ekf.fallback_active && (
          <div className="badge-neon border border-neon-crimson/50 bg-neon-crimson/15 text-neon-crimson animate-pulse px-3 py-1">
            <AlertTriangle className="w-3.5 h-3.5" />
            <span className="font-bold text-xs">AUTONOMOUS FALLBACK</span>
          </div>
        )}

        {/* Rover Mission Status */}
        <div className="hidden sm:flex items-center gap-2 px-3 py-1 rounded-lg bg-space-850 border border-slate-800 font-mono text-xs text-slate-300">
          <Compass className="w-3.5 h-3.5 text-neon-emerald" />
          <span>STATUS:</span>
          <span className="font-bold text-neon-emerald">
            {state?.rover.mission_status || 'INITIALIZING'}
          </span>
        </div>
      </div>

      {/* Right Controls & Clocks */}
      <div className="flex items-center gap-3">
        {/* Mission Elapsed Time */}
        <div className="bg-space-850 border border-slate-800 px-3 py-1 rounded-lg font-mono text-xs flex items-center gap-2">
          <Activity className="w-3.5 h-3.5 text-neon-cyan" />
          <span className="text-slate-400">MET:</span>
          <span className="text-neon-cyan font-bold tracking-wider">{metString}</span>
        </div>

        {/* UTC Clock */}
        <div className="hidden md:block bg-space-850 border border-slate-800 px-3 py-1 rounded-lg font-mono text-xs text-slate-400">
          {utcTime}
        </div>

        {/* Benchmark Compare Button */}
        <button
          onClick={onOpenBenchmark}
          className="btn-aerospace bg-gradient-to-r from-purple-600/30 to-indigo-600/30 hover:from-purple-600/50 hover:to-indigo-600/50 text-purple-200 border border-purple-500/40 shadow-sm"
          title="Run head-to-head evaluation benchmark across Mode A, Mode B, and Mode C"
        >
          <ShieldCheck className="w-3.5 h-3.5 text-purple-400" />
          <span>COMPARE MODES</span>
        </button>

        {/* WebSocket Connection Light */}
        <div className="flex items-center gap-1.5 px-2 py-1 rounded bg-space-850 border border-slate-800 text-[11px] font-mono">
          <span className={`w-2 h-2 rounded-full ${connected ? 'bg-neon-emerald shadow-neon-emerald animate-pulse' : 'bg-neon-crimson'}`} />
          <span className="text-slate-400">{connected ? 'LINK 20Hz' : 'DISCONNECTED'}</span>
        </div>
      </div>
    </header>
  );
};
