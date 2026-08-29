import React from 'react';
import { Activity, Battery, Gauge, Compass, MapPin, Radio, ShieldCheck, Zap } from 'lucide-react';
import { RoverState, PerformanceMetricsState } from '../types/simulation';

interface TelemetryPanelProps {
  rover: RoverState | null;
  metrics: PerformanceMetricsState | null;
}

export const TelemetryPanel: React.FC<TelemetryPanelProps> = ({ rover, metrics }) => {
  const batteryPct = rover?.battery_pct || 100;
  const speed = rover?.speed || 0.0;
  const headingDeg = rover?.heading_deg || 0;
  const distToTarget = rover?.dist_to_target || 0;
  const totalDist = rover?.total_distance || 0;
  const wheelSlip = rover?.wheel_slip ? (rover.wheel_slip * 100).toFixed(1) : '2.0';

  const getBatteryColor = (pct: number) => {
    if (pct > 60) return 'text-neon-emerald';
    if (pct > 25) return 'text-amber-400';
    return 'text-neon-crimson';
  };

  return (
    <div className="panel-glass flex flex-col h-full overflow-hidden">
      {/* Panel Header */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <Activity className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">ROVER TELEMETRY & FLIGHT STATS</span>
        </div>
        <span className="text-[10px] font-mono text-neon-emerald">AUTONOMOUS GUIDANCE</span>
      </div>

      <div className="p-3 grid grid-cols-2 sm:grid-cols-4 gap-2.5 font-mono text-xs">
        {/* Position */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1 flex items-center gap-1">
            <MapPin className="w-3 h-3 text-neon-cyan" /> COORDINATES (X, Y)
          </div>
          <div className="text-base font-bold text-white">
            {rover?.x.toFixed(1)}m, {rover?.y.toFixed(1)}m
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Mars Datum 0,0</div>
        </div>

        {/* Speed */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1 flex items-center gap-1">
            <Gauge className="w-3 h-3 text-neon-emerald" /> ROVER SPEED
          </div>
          <div className="text-base font-bold text-white flex items-baseline gap-1">
            <span>{speed.toFixed(2)}</span>
            <span className="text-xs text-slate-400">m/s</span>
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Max: 3.5 m/s</div>
        </div>

        {/* Heading */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1 flex items-center gap-1">
            <Compass className="w-3 h-3 text-amber-400" /> HEADING AZIMUTH
          </div>
          <div className="text-base font-bold text-white">
            {headingDeg.toFixed(1)}°
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Yaw Angle</div>
        </div>

        {/* Battery */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1 flex items-center gap-1">
            <Battery className="w-3 h-3 text-neon-cyan" /> POWER PACK
          </div>
          <div className={`text-base font-bold flex items-baseline gap-1 ${getBatteryColor(batteryPct)}`}>
            <span>{batteryPct.toFixed(1)}</span>
            <span className="text-xs text-slate-400">%</span>
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Solar Assisted</div>
        </div>

        {/* Distance Traveled */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1">TRAVELED DISTANCE</div>
          <div className="text-base font-bold text-slate-200">{totalDist.toFixed(1)} m</div>
          <div className="text-[9px] text-slate-500 mt-1">Odometer Ticks</div>
        </div>

        {/* Distance to Science Target */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1">TARGET RANGE</div>
          <div className="text-base font-bold text-neon-emerald">{distToTarget.toFixed(1)} m</div>
          <div className="text-[9px] text-slate-500 mt-1">To Target (540, 530)</div>
        </div>

        {/* Satellite Coverage % */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1 flex items-center gap-1">
            <Radio className="w-3 h-3 text-neon-cyan" /> COVERAGE TIME
          </div>
          <div className="text-base font-bold text-neon-cyan">
            {metrics?.coverage_pct.toFixed(1) || '100.0'}%
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Constellation Window</div>
        </div>

        {/* Wheel Slip Factor */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800">
          <div className="text-[10px] text-slate-400 mb-1">WHEEL SLIP FACTOR</div>
          <div className={`text-base font-bold ${Number(wheelSlip) > 15 ? 'text-amber-400' : 'text-slate-200'}`}>
            {wheelSlip}%
          </div>
          <div className="text-[9px] text-slate-500 mt-1">Terrain Friction Index</div>
        </div>
      </div>
    </div>
  );
};
