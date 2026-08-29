import React, { useState, useEffect } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  AreaChart,
  Area,
} from 'recharts';
import { LineChart as ChartIcon, Activity, TrendingUp, ShieldAlert, Cpu } from 'lucide-react';
import { PerformanceMetricsState } from '../types/simulation';

interface RealtimeChartsProps {
  metrics: PerformanceMetricsState | null;
  simTime: number;
}

interface ChartSample {
  time: number;
  posError: number;
  uncertainty: number;
  activeSats: number;
}

export const RealtimeCharts: React.FC<RealtimeChartsProps> = ({ metrics, simTime }) => {
  const [history, setHistory] = useState<ChartSample[]>([]);
  const [selectedChart, setSelectedChart] = useState<'ERROR_UNCERTAINTY' | 'SATELLITE_USAGE'>('ERROR_UNCERTAINTY');

  useEffect(() => {
    if (!metrics) return;

    const sample: ChartSample = {
      time: Math.round(simTime * 10) / 10,
      posError: metrics.position_error_instant_m || metrics.current_position_error_m || 0.0,
      uncertainty: metrics.current_uncertainty_m || 0.5,
      activeSats: metrics.active_satellites_count || 0,
    };

    setHistory((prev) => {
      const updated = [...prev, sample];
      return updated.length > 50 ? updated.slice(updated.length - 50) : updated;
    });
  }, [metrics, simTime]);

  return (
    <div className="panel-glass flex flex-col h-full overflow-hidden">
      {/* Chart Header */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <ChartIcon className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">REAL-TIME PERFORMANCE TELEMETRY</span>
        </div>
        {/* Toggle Charts */}
        <div className="flex items-center gap-1 font-mono text-[10px]">
          <button
            onClick={() => setSelectedChart('ERROR_UNCERTAINTY')}
            className={`px-2 py-0.5 rounded transition-all ${
              selectedChart === 'ERROR_UNCERTAINTY'
                ? 'bg-neon-cyan/20 text-neon-cyan font-bold border border-neon-cyan/30'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            ERROR vs UNCERTAINTY
          </button>
          <button
            onClick={() => setSelectedChart('SATELLITE_USAGE')}
            className={`px-2 py-0.5 rounded transition-all ${
              selectedChart === 'SATELLITE_USAGE'
                ? 'bg-purple-500/20 text-purple-300 font-bold border border-purple-500/30'
                : 'text-slate-400 hover:text-slate-200'
            }`}
          >
            SATELLITE UTILIZATION
          </button>
        </div>
      </div>

      {/* Chart Canvas Area */}
      <div className="p-3 flex-1 min-h-[160px] w-full">
        {selectedChart === 'ERROR_UNCERTAINTY' ? (
          <ResponsiveContainer width="100%" height={160}>
            <AreaChart data={history} margin={{ top: 5, right: 10, left: -20, bottom: 0 }}>
              <defs>
                <linearGradient id="errorGrad" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#ef4444" stopOpacity={0.4} />
                  <stop offset="95%" stopColor="#ef4444" stopOpacity={0} />
                </linearGradient>
                <linearGradient id="uncGrad" x1="0" y1="0" x2="0" y2="1">
                  <stop offset="5%" stopColor="#00f0ff" stopOpacity={0.3} />
                  <stop offset="95%" stopColor="#00f0ff" stopOpacity={0} />
                </linearGradient>
              </defs>
              <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
              <XAxis dataKey="time" stroke="#64748b" tick={{ fontSize: 10, fill: '#64748b' }} unit="s" />
              <YAxis stroke="#64748b" tick={{ fontSize: 10, fill: '#64748b' }} unit="m" />
              <Tooltip
                contentStyle={{
                  backgroundColor: '#080d1a',
                  borderColor: '#334155',
                  borderRadius: '8px',
                  fontSize: '11px',
                  fontFamily: 'monospace',
                }}
              />
              <Area
                type="monotone"
                dataKey="posError"
                name="Position Error (m)"
                stroke="#ef4444"
                strokeWidth={2}
                fillOpacity={1}
                fill="url(#errorGrad)"
                isAnimationActive={false}
              />
              <Area
                type="monotone"
                dataKey="uncertainty"
                name="Uncertainty (2σ)"
                stroke="#00f0ff"
                strokeWidth={2}
                fillOpacity={1}
                fill="url(#uncGrad)"
                isAnimationActive={false}
              />
            </AreaChart>
          </ResponsiveContainer>
        ) : (
          <ResponsiveContainer width="100%" height={160}>
            <LineChart data={history} margin={{ top: 5, right: 10, left: -20, bottom: 0 }}>
              <CartesianGrid strokeDasharray="3 3" stroke="#1e293b" />
              <XAxis dataKey="time" stroke="#64748b" tick={{ fontSize: 10, fill: '#64748b' }} unit="s" />
              <YAxis domain={[0, 4]} stroke="#64748b" tick={{ fontSize: 10, fill: '#64748b' }} />
              <Tooltip
                contentStyle={{
                  backgroundColor: '#080d1a',
                  borderColor: '#334155',
                  borderRadius: '8px',
                  fontSize: '11px',
                  fontFamily: 'monospace',
                }}
              />
              <Line
                type="stepAfter"
                dataKey="activeSats"
                name="Active Orbiters"
                stroke="#a855f7"
                strokeWidth={2.5}
                dot={false}
                isAnimationActive={false}
              />
            </LineChart>
          </ResponsiveContainer>
        )}
      </div>
    </div>
  );
};
