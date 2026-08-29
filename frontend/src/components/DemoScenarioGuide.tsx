import React from 'react';
import { Bot, CheckCircle2, ChevronRight, Play, Sparkles } from 'lucide-react';
import { DemoMissionStatus } from '../types/simulation';

interface DemoScenarioGuideProps {
  demoStatus: DemoMissionStatus;
  onStartDemo: () => void;
}

export const DemoScenarioGuide: React.FC<DemoScenarioGuideProps> = ({ demoStatus, onStartDemo }) => {
  if (!demoStatus.is_active) {
    return null;
  }

  const currentStage = demoStatus.stages[demoStatus.current_stage_idx] || {
    id: '01',
    name: 'Initializing Demonstration',
    desc: 'Setting up deterministic mission scenario...',
  };

  const progressPct = ((demoStatus.current_stage_idx + 1) / demoStatus.stages.length) * 100;

  return (
    <div className="w-full panel-glass p-3 border-neon-cyan/40 bg-gradient-to-r from-space-900 via-cyan-950/20 to-space-900 shadow-neon-cyan/20 animate-fade-in">
      <div className="flex flex-wrap items-center justify-between gap-3 font-mono text-xs">
        {/* Left: Milestone Counter */}
        <div className="flex items-center gap-2.5">
          <div className="flex items-center justify-center w-7 h-7 rounded-lg bg-neon-cyan/20 border border-neon-cyan/40 text-neon-cyan font-bold">
            {currentStage.id}
          </div>
          <div>
            <div className="flex items-center gap-2">
              <span className="font-bold text-neon-cyan tracking-wider">
                NAVIS DEMO MISSION STAGE {demoStatus.current_stage_idx + 1}/{demoStatus.stages.length}
              </span>
              <span className="text-[10px] text-slate-400">({demoStatus.demo_time_s.toFixed(1)}s)</span>
            </div>
            <div className="text-white font-semibold text-xs mt-0.5">{currentStage.name}</div>
          </div>
        </div>

        {/* Center: Stage Description */}
        <div className="text-slate-300 text-xs flex-1 max-w-xl truncate hidden md:block">
          {currentStage.desc}
        </div>

        {/* Right: Progress Indicator */}
        <div className="flex items-center gap-3">
          <div className="w-32 bg-slate-800 rounded-full h-2 overflow-hidden border border-slate-700">
            <div
              className="bg-gradient-to-r from-neon-cyan to-blue-400 h-full transition-all duration-300 shadow-neon-cyan"
              style={{ width: `${progressPct}%` }}
            />
          </div>
          <span className="text-neon-cyan font-bold text-xs">{progressPct.toFixed(0)}%</span>
        </div>
      </div>
    </div>
  );
};
