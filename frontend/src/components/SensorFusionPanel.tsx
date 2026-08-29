import React from 'react';
import { Layers, ShieldCheck, AlertTriangle, Compass, Target, Gauge, Cpu, Check, X } from 'lucide-react';
import { EKFState, SensorSuiteState, PerformanceMetricsState } from '../types/simulation';

interface SensorFusionPanelProps {
  ekf: EKFState | null;
  sensors: SensorSuiteState | null;
  metrics: PerformanceMetricsState | null;
  truePos: { x: number; y: number; speed: number; heading_deg: number };
}

export const SensorFusionPanel: React.FC<SensorFusionPanelProps> = ({
  ekf,
  sensors,
  metrics,
  truePos,
}) => {
  const isFallback = ekf?.fallback_active || false;
  const pntActive = sensors?.pnt?.active || false;
  const posError = metrics?.position_error_instant_m || metrics?.current_position_error_m || 0.0;
  const uncertainty = ekf?.uncertainty_m || 0.5;

  return (
    <div className="panel-glass flex flex-col h-full overflow-hidden">
      {/* Panel Header */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <Layers className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">SENSOR FUSION & EKF ESTIMATOR</span>
        </div>
        <div className="flex items-center gap-1.5 font-mono text-[10px]">
          {isFallback ? (
            <span className="text-neon-crimson flex items-center gap-1">
              <AlertTriangle className="w-3 h-3" /> FALLBACK ONBOARD
            </span>
          ) : (
            <span className="text-neon-emerald flex items-center gap-1">
              <ShieldCheck className="w-3 h-3" /> EKF PNT FUSED
            </span>
          )}
        </div>
      </div>

      <div className="p-3 flex flex-col gap-3">
        {/* 4 Sensor Status Badges */}
        <div className="grid grid-cols-2 sm:grid-cols-4 gap-2">
          {/* Satellite PNT */}
          <div
            className={`p-2 rounded-lg border font-mono text-[11px] flex items-center justify-between ${
              pntActive
                ? 'bg-neon-cyan/10 border-neon-cyan/30 text-neon-cyan'
                : 'bg-neon-crimson/10 border-neon-crimson/30 text-neon-crimson'
            }`}
          >
            <span>PNT</span>
            {pntActive ? <Check className="w-3.5 h-3.5" /> : <X className="w-3.5 h-3.5" />}
          </div>

          {/* IMU */}
          <div className="p-2 rounded-lg border bg-neon-emerald/10 border-neon-emerald/30 text-neon-emerald font-mono text-[11px] flex items-center justify-between">
            <span>IMU</span>
            <Check className="w-3.5 h-3.5" />
          </div>

          {/* Visual Odometry */}
          <div className="p-2 rounded-lg border bg-neon-emerald/10 border-neon-emerald/30 text-neon-emerald font-mono text-[11px] flex items-center justify-between">
            <span>CAMERA</span>
            <Check className="w-3.5 h-3.5" />
          </div>

          {/* Wheel Odometry */}
          <div className="p-2 rounded-lg border bg-neon-emerald/10 border-neon-emerald/30 text-neon-emerald font-mono text-[11px] flex items-center justify-between">
            <span>ODOMETRY</span>
            <Check className="w-3.5 h-3.5" />
          </div>
        </div>

        {/* Fallback Banner Alert */}
        {isFallback && (
          <div className="p-2 rounded-lg bg-neon-crimson/15 border border-neon-crimson/40 text-neon-crimson font-mono text-xs flex items-center gap-2 animate-pulse">
            <AlertTriangle className="w-4 h-4 shrink-0" />
            <div>
              <div className="font-bold">AUTONOMOUS FALLBACK ACTIVE</div>
              <div className="text-[10px] text-red-300">PNT lost. Propagating dead-reckoning via IMU + VO + Wheel Odometry.</div>
            </div>
          </div>
        )}

        {/* EKF Position & Covariance Metrics */}
        <div className="grid grid-cols-2 gap-2">
          {/* Position Error */}
          <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800 font-mono">
            <div className="text-[10px] text-slate-400 mb-0.5 flex items-center gap-1">
              <Target className="w-3 h-3 text-neon-cyan" /> POSITION ERROR
            </div>
            <div className="text-lg font-bold text-slate-100 flex items-baseline gap-1">
              <span>{posError.toFixed(2)}</span>
              <span className="text-xs text-slate-400">m</span>
            </div>
            <div className="text-[9px] text-slate-500 mt-1">|True - Estimated|</div>
          </div>

          {/* Navigation Uncertainty */}
          <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800 font-mono">
            <div className="text-[10px] text-slate-400 mb-0.5 flex items-center gap-1">
              <Gauge className="w-3 h-3 text-neon-emerald" /> UNCERTAINTY (2σ)
            </div>
            <div className={`text-lg font-bold flex items-baseline gap-1 ${uncertainty > 2.5 ? 'text-amber-400' : 'text-neon-emerald'}`}>
              <span>{uncertainty.toFixed(2)}</span>
              <span className="text-xs text-slate-400">m</span>
            </div>
            <div className="text-[9px] text-slate-500 mt-1">EKF Covariance Trace</div>
          </div>
        </div>

        {/* State Vector Readouts */}
        <div className="p-2.5 rounded-xl bg-space-850 border border-slate-800 font-mono text-xs space-y-1.5">
          <div className="text-[10px] font-bold text-slate-400 border-b border-slate-700/60 pb-1">
            ESTIMATED STATE VECTOR (8-DOF)
          </div>
          <div className="flex justify-between">
            <span className="text-slate-400">Est Coordinates (X, Y):</span>
            <span className="text-neon-cyan font-bold">
              {ekf?.est_x.toFixed(1)}m, {ekf?.est_y.toFixed(1)}m
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-slate-400">Est Velocity (Vx, Vy):</span>
            <span className="text-slate-200">
              {ekf?.est_vx.toFixed(2)}m/s, {ekf?.est_vy.toFixed(2)}m/s
            </span>
          </div>
          <div className="flex justify-between">
            <span className="text-slate-400">Estimated Heading:</span>
            <span className="text-slate-200">{ekf?.est_heading_deg.toFixed(1)}°</span>
          </div>
          <div className="flex justify-between text-[10px] text-slate-500 pt-1 border-t border-slate-800">
            <span>IMU Acc Bias:</span>
            <span>{ekf?.bias_estimates.b_ax.toFixed(3)} m/s²</span>
          </div>
        </div>
      </div>
    </div>
  );
};
