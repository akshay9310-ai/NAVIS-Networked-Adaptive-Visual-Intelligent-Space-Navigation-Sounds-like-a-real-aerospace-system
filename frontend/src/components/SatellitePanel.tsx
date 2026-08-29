import React from 'react';
import { Satellite, Radio, Camera, Cpu, Battery, Eye, CheckCircle2, XCircle, Sparkles } from 'lucide-react';
import { SatelliteState, AISchedulerState } from '../types/simulation';

interface SatellitePanelProps {
  satellites: SatelliteState[];
  aiScheduler: AISchedulerState;
}

export const SatellitePanel: React.FC<SatellitePanelProps> = ({ satellites, aiScheduler }) => {
  const getTaskBadge = (task: string, isVisible: boolean) => {
    if (!isVisible || task === 'IDLE') {
      return (
        <span className="px-2 py-0.5 rounded text-[10px] font-mono bg-slate-800 text-slate-500 border border-slate-700">
          IDLE
        </span>
      );
    }
    if (task === 'PNT') {
      return (
        <span className="px-2 py-0.5 rounded text-[10px] font-mono font-bold bg-neon-cyan/15 text-neon-cyan border border-neon-cyan/40 flex items-center gap-1 shadow-neon-cyan/20">
          <Radio className="w-3 h-3" /> PNT LOCK
        </span>
      );
    }
    if (task === 'IMAGING') {
      return (
        <span className="px-2 py-0.5 rounded text-[10px] font-mono font-bold bg-purple-500/15 text-purple-300 border border-purple-500/40 flex items-center gap-1">
          <Camera className="w-3 h-3" /> IMAGING
        </span>
      );
    }
    return (
      <span className="px-2 py-0.5 rounded text-[10px] font-mono font-bold bg-neon-emerald/15 text-neon-emerald border border-neon-emerald/40 flex items-center gap-1">
        <Satellite className="w-3 h-3" /> COMM RELAY
      </span>
    );
  };

  return (
    <div className="panel-glass flex flex-col h-full overflow-hidden">
      {/* Header */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <Satellite className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">SATELLITE NETWORK</span>
        </div>
        <span className="text-[10px] font-mono text-neon-cyan">
          {satellites.filter((s) => s.is_visible).length}/4 IN VIEW
        </span>
      </div>

      {/* Constellation Grid List */}
      <div className="p-3 grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-2.5">
        {satellites.map((sat) => {
          const isSelected = aiScheduler.selected_satellite === sat.sat_id;
          const score = aiScheduler.scores?.[sat.sat_id] || sat.score || 0;

          return (
            <div
              key={sat.sat_id}
              className={`p-2.5 rounded-xl border transition-all relative ${
                sat.is_outage
                  ? 'bg-neon-crimson/5 border-neon-crimson/30'
                  : sat.is_visible
                  ? isSelected
                    ? 'bg-space-850 border-neon-cyan/50 shadow-neon-cyan/20'
                    : 'bg-space-850 border-slate-700/60'
                  : 'bg-space-900/50 border-slate-800/60 opacity-65'
              }`}
            >
              {/* Star Badge if primary selected asset */}
              {isSelected && sat.is_visible && (
                <div className="absolute -top-2 -right-1 px-1.5 py-0.2 rounded-full bg-neon-cyan text-space-950 text-[9px] font-mono font-bold flex items-center gap-0.5 shadow-sm">
                  <Sparkles className="w-2.5 h-2.5" /> AI PRIORITY
                </div>
              )}

              {/* Title & Task */}
              <div className="flex items-center justify-between gap-1 mb-1.5">
                <div className="flex items-center gap-1.5">
                  <span className="font-mono font-bold text-xs text-white">{sat.sat_id}</span>
                  <span className="text-[10px] text-slate-400 font-mono">({sat.name})</span>
                </div>
                {getTaskBadge(sat.current_task, sat.is_visible)}
              </div>

              {/* Visibility & Elevation */}
              <div className="flex items-center justify-between text-[11px] font-mono mb-1">
                <div className="flex items-center gap-1">
                  {sat.is_outage ? (
                    <span className="text-neon-crimson flex items-center gap-1">
                      <XCircle className="w-3 h-3" /> OUTAGE
                    </span>
                  ) : sat.is_visible ? (
                    <span className="text-neon-emerald flex items-center gap-1">
                      <CheckCircle2 className="w-3 h-3" /> VISIBLE
                    </span>
                  ) : (
                    <span className="text-slate-500 flex items-center gap-1">
                      <XCircle className="w-3 h-3" /> NOT VISIBLE
                    </span>
                  )}
                </div>
                <span className="text-slate-400">ELEV: {sat.elevation_deg}°</span>
              </div>

              {/* Altitude & Battery */}
              <div className="flex items-center justify-between text-[10px] font-mono text-slate-400 mb-2">
                <span>ALT: {sat.altitude_km} km</span>
                <div className="flex items-center gap-1">
                  <Battery className="w-3 h-3 text-slate-400" />
                  <span>{sat.battery_pct.toFixed(0)}%</span>
                </div>
              </div>

              {/* AI Dynamic Score Meter */}
              <div className="w-full bg-slate-800/80 rounded-full h-1.5 overflow-hidden mb-1">
                <div
                  className={`h-full transition-all duration-300 ${
                    sat.current_task === 'PNT'
                      ? 'bg-neon-cyan'
                      : sat.current_task === 'IMAGING'
                      ? 'bg-purple-400'
                      : 'bg-neon-emerald'
                  }`}
                  style={{ width: `${Math.min(100, Math.max(5, score * 100))}%` }}
                />
              </div>
              <div className="flex items-center justify-between text-[9px] font-mono text-slate-500">
                <span>AI MATCH SCORE</span>
                <span className="text-slate-300 font-bold">{score.toFixed(2)}</span>
              </div>
            </div>
          );
        })}
      </div>

      {/* AI Scheduler Explainability Card */}
      {aiScheduler.reason_summary && (
        <div className="mx-3 mb-3 p-2 rounded-lg bg-neon-cyan/5 border border-neon-cyan/20 flex items-start gap-2 text-[11px] font-mono">
          <Cpu className="w-4 h-4 text-neon-cyan shrink-0 mt-0.5" />
          <div>
            <div className="text-neon-cyan font-semibold flex items-center gap-1.5">
              <span>AI SCHEDULER DECISION MATRIX</span>
            </div>
            <p className="text-slate-300 mt-0.5">{aiScheduler.reason_summary}</p>
          </div>
        </div>
      )}
    </div>
  );
};
