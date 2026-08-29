import React, { useState, useRef, useEffect } from 'react';
import { Terminal, Filter, ShieldAlert, Cpu, Route, Sparkles } from 'lucide-react';
import { MissionLogEntry } from '../types/simulation';

interface AIDecisionConsoleProps {
  logs: MissionLogEntry[];
}

export const AIDecisionConsole: React.FC<AIDecisionConsoleProps> = ({ logs }) => {
  const [filter, setFilter] = useState<'ALL' | 'HAZARD' | 'SCHEDULER' | 'PLANNER' | 'OUTAGE'>('ALL');
  const scrollRef = useRef<HTMLDivElement | null>(null);

  // Auto-scroll to bottom when new logs arrive
  useEffect(() => {
    if (scrollRef.current) {
      scrollRef.current.scrollTop = scrollRef.current.scrollHeight;
    }
  }, [logs]);

  const filteredLogs = logs.filter((log) => {
    if (filter === 'ALL') return true;
    return log.category.toUpperCase().includes(filter);
  });

  const getBadgeStyle = (level: string) => {
    switch (level) {
      case 'danger':
        return 'text-neon-crimson bg-neon-crimson/10 border-neon-crimson/30';
      case 'warning':
        return 'text-amber-400 bg-amber-500/10 border-amber-500/30';
      case 'success':
        return 'text-neon-emerald bg-neon-emerald/10 border-neon-emerald/30';
      default:
        return 'text-neon-cyan bg-neon-cyan/10 border-neon-cyan/30';
    }
  };

  return (
    <div className="panel-glass flex flex-col h-full overflow-hidden">
      {/* Console Header & Filter Bar */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <Terminal className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">AI DECISION & EVENT LOG</span>
        </div>
        {/* Filters */}
        <div className="flex items-center gap-1 font-mono text-[10px]">
          {(['ALL', 'HAZARD', 'SCHEDULER', 'PLANNER', 'OUTAGE'] as const).map((f) => (
            <button
              key={f}
              onClick={() => setFilter(f)}
              className={`px-1.5 py-0.5 rounded transition-all ${
                filter === f
                  ? 'bg-neon-cyan/20 text-neon-cyan font-bold border border-neon-cyan/30'
                  : 'text-slate-400 hover:text-slate-200'
              }`}
            >
              {f}
            </button>
          ))}
        </div>
      </div>

      {/* Terminal Output Area */}
      <div
        ref={scrollRef}
        className="p-3 flex-1 overflow-y-auto space-y-2 font-mono text-xs max-h-[220px] bg-space-950/60"
      >
        {filteredLogs.length === 0 ? (
          <div className="text-slate-500 text-center py-6">Awaiting telemetry & decision events...</div>
        ) : (
          filteredLogs.map((log) => (
            <div
              key={log.id}
              className="flex items-start gap-2 leading-relaxed border-b border-slate-800/40 pb-1.5"
            >
              <span className="text-slate-500 text-[10px] shrink-0 mt-0.5">[{log.time}]</span>
              <span className="text-slate-400 text-[10px] shrink-0 mt-0.5">{log.met}</span>
              <span className={`px-1.5 py-0.2 rounded text-[9px] font-bold border shrink-0 ${getBadgeStyle(log.level)}`}>
                {log.category}
              </span>
              <span
                className={`text-[11px] ${
                  log.level === 'danger'
                    ? 'text-red-300 font-semibold'
                    : log.level === 'warning'
                    ? 'text-amber-200'
                    : log.level === 'success'
                    ? 'text-emerald-300 font-semibold'
                    : 'text-slate-300'
                }`}
              >
                {log.message}
              </span>
            </div>
          ))
        )}
      </div>
    </div>
  );
};
