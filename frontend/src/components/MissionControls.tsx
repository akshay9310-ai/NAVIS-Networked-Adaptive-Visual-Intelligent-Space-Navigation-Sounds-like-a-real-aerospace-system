import React from 'react';
import { Play, Pause, RotateCcw, Zap, ZapOff, AlertOctagon, Bot, FastForward, Cpu } from 'lucide-react';
import { SimulationFullState } from '../types/simulation';

interface MissionControlsProps {
  state: SimulationFullState | null;
  onStart: () => void;
  onPause: () => void;
  onReset: () => void;
  onSpeed: (multiplier: number) => void;
  onMode: (mode: 'MODE_A' | 'MODE_B' | 'MODE_C') => void;
  onOutage: () => void;
  onRestore: () => void;
  onInjectHazard: () => void;
  onRunDemo: () => void;
}

export const MissionControls: React.FC<MissionControlsProps> = ({
  state,
  onStart,
  onPause,
  onReset,
  onSpeed,
  onMode,
  onOutage,
  onRestore,
  onInjectHazard,
  onRunDemo,
}) => {
  const isRunning = state?.is_running || false;
  const isOutage = state?.satellites.some((s) => s.is_outage) || false;
  const currentSpeed = state?.speed_multiplier || 1.0;
  const currentMode = state?.active_mode || 'MODE_C';

  return (
    <div className="w-full panel-glass p-3 flex flex-wrap items-center justify-between gap-3">
      {/* Primary Simulation Flow Controls */}
      <div className="flex items-center gap-2">
        {!isRunning ? (
          <button
            onClick={onStart}
            className="btn-aerospace bg-neon-emerald/20 hover:bg-neon-emerald/30 text-neon-emerald border border-neon-emerald/40 shadow-neon-emerald"
          >
            <Play className="w-4 h-4 fill-current" />
            <span>START MISSION</span>
          </button>
        ) : (
          <button
            onClick={onPause}
            className="btn-aerospace bg-amber-500/20 hover:bg-amber-500/30 text-amber-400 border border-amber-500/40"
          >
            <Pause className="w-4 h-4 fill-current" />
            <span>PAUSE</span>
          </button>
        )}

        <button
          onClick={onReset}
          className="btn-aerospace bg-space-800 hover:bg-space-700 text-slate-300 border border-slate-700"
          title="Reset Simulation to Initial State"
        >
          <RotateCcw className="w-4 h-4" />
          <span>RESET</span>
        </button>

        {/* Demo Mission Trigger Button */}
        <button
          onClick={onRunDemo}
          className="btn-aerospace bg-gradient-to-r from-cyan-600/30 to-blue-600/30 hover:from-cyan-600/50 hover:to-blue-600/50 text-cyan-300 border border-neon-cyan/40 shadow-neon-cyan"
          title="Automated 15-Stage NAVIS Capability Demonstration"
        >
          <Bot className="w-4 h-4 text-neon-cyan animate-bounce" />
          <span>RUN DEMO MISSION</span>
        </button>
      </div>

      {/* Outage & Hazard Injection Tools */}
      <div className="flex items-center gap-2">
        {!isOutage ? (
          <button
            onClick={onOutage}
            className="btn-aerospace bg-neon-crimson/15 hover:bg-neon-crimson/30 text-neon-crimson border border-neon-crimson/40"
            title="Simulate complete satellite constellation PNT signal loss"
          >
            <ZapOff className="w-4 h-4" />
            <span>SIMULATE SATELLITE OUTAGE</span>
          </button>
        ) : (
          <button
            onClick={onRestore}
            className="btn-aerospace bg-neon-emerald/20 hover:bg-neon-emerald/30 text-neon-emerald border border-neon-emerald/40 animate-pulse"
            title="Re-establish satellite constellation PNT lock"
          >
            <Zap className="w-4 h-4" />
            <span>RESTORE SATELLITE</span>
          </button>
        )}

        <button
          onClick={onInjectHazard}
          className="btn-aerospace bg-amber-500/15 hover:bg-amber-500/25 text-amber-400 border border-amber-500/40"
          title="Dynamically spawn an obstacle to test A* replanning"
        >
          <AlertOctagon className="w-4 h-4" />
          <span>INJECT HAZARD</span>
        </button>
      </div>

      {/* Mode Selector (Mode A, B, C) */}
      <div className="flex items-center gap-1 bg-space-850 p-1 rounded-lg border border-slate-800 font-mono text-xs">
        <span className="text-slate-500 px-2 flex items-center gap-1">
          <Cpu className="w-3 h-3" /> MODE:
        </span>
        {(['MODE_A', 'MODE_B', 'MODE_C'] as const).map((mode) => (
          <button
            key={mode}
            onClick={() => onMode(mode)}
            className={`px-2.5 py-1 rounded font-bold transition-all ${
              currentMode === mode
                ? mode === 'MODE_C'
                  ? 'bg-neon-cyan/20 text-neon-cyan border border-neon-cyan/40 shadow-neon-cyan'
                  : 'bg-slate-700 text-white border border-slate-600'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            {mode === 'MODE_A' ? 'A: ROVER ONLY' : mode === 'MODE_B' ? 'B: FIXED SAT' : 'C: NAVIS AI'}
          </button>
        ))}
      </div>

      {/* Speed Multiplier Controls */}
      <div className="flex items-center gap-1 bg-space-850 p-1 rounded-lg border border-slate-800 font-mono text-xs">
        <span className="text-slate-500 px-1.5 flex items-center gap-1">
          <FastForward className="w-3 h-3" /> SPEED:
        </span>
        {[0.5, 1.0, 2.0, 5.0].map((spd) => (
          <button
            key={spd}
            onClick={() => onSpeed(spd)}
            className={`px-2 py-0.5 rounded transition-all ${
              currentSpeed === spd
                ? 'bg-neon-cyan/20 text-neon-cyan font-bold border border-neon-cyan/30'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            {spd}×
          </button>
        ))}
      </div>
    </div>
  );
};
