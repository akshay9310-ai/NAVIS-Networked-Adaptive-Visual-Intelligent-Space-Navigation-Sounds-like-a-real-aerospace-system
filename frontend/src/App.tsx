import React, { useState } from 'react';
import { useSimulation } from './hooks/useSimulation';
import { Header } from './components/Header';
import { MarsCanvas } from './components/MarsCanvas';
import { MissionControls } from './components/MissionControls';
import { SatellitePanel } from './components/SatellitePanel';
import { SensorFusionPanel } from './components/SensorFusionPanel';
import { AIDecisionConsole } from './components/AIDecisionConsole';
import { TelemetryPanel } from './components/TelemetryPanel';
import { RealtimeCharts } from './components/RealtimeCharts';
import { BenchmarkModal } from './components/BenchmarkModal';
import { DemoScenarioGuide } from './components/DemoScenarioGuide';

export function App() {
  const {
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
  } = useSimulation();

  const [isBenchmarkModalOpen, setIsBenchmarkModalOpen] = useState(false);

  const handleOpenBenchmark = () => {
    setIsBenchmarkModalOpen(true);
    if (!benchmarkResult) {
      runBenchmark();
    }
  };

  const handleManualInjectHazard = () => {
    if (!state) return;
    // Inject in front of rover along forward route
    const roverX = state.rover.x;
    const roverY = state.rover.y;
    const offset = 80;
    const hzX = Math.min(state.terrain_summary.width_m - 50, roverX + offset);
    const hzY = Math.min(state.terrain_summary.height_m - 50, roverY + offset);
    injectHazard(hzX, hzY, 28.0, 'ROCK_FIELD');
  };

  return (
    <div className="min-h-screen w-full bg-space-950 flex flex-col font-sans text-slate-100 antialiased selection:bg-neon-cyan/20 selection:text-neon-cyan overflow-x-hidden">
      {/* Top Header */}
      <Header
        state={state}
        connected={connected}
        onOpenBenchmark={handleOpenBenchmark}
      />

      {/* Main Container */}
      <main className="flex-1 w-full max-w-[1920px] mx-auto p-3 flex flex-col gap-3">
        {/* Demo Mission HUD Milestone Tracker (if demo active) */}
        {state?.demo_mission && (
          <DemoScenarioGuide
            demoStatus={state.demo_mission}
            onStartDemo={runDemo}
          />
        )}

        {/* Global Mission Controls Bar */}
        <MissionControls
          state={state}
          onStart={startMission}
          onPause={pauseMission}
          onReset={resetMission}
          onSpeed={setSpeed}
          onMode={setMode}
          onOutage={simulateOutage}
          onRestore={restoreSatellite}
          onInjectHazard={handleManualInjectHazard}
          onRunDemo={runDemo}
        />

        {/* Central Tactical Grid */}
        <div className="grid grid-cols-1 lg:grid-cols-12 gap-3 flex-1 min-h-[500px]">
          {/* Main Visual Center: Interactive Mars Canvas */}
          <div className="lg:col-span-8 flex flex-col min-h-[480px]">
            <MarsCanvas
              state={state}
              onInjectHazard={(x, y, rad, type) => injectHazard(x, y, rad, type)}
            />
          </div>

          {/* Right Column: Satellite Constellation & Sensor Fusion */}
          <div className="lg:col-span-4 flex flex-col gap-3">
            {/* Satellite Constellation & AI Scheduler */}
            <div className="flex-1 min-h-[260px]">
              <SatellitePanel
                satellites={state?.satellites || []}
                aiScheduler={state?.ai_scheduler || {}}
              />
            </div>

            {/* Sensor Fusion & EKF */}
            <div className="flex-1 min-h-[260px]">
              <SensorFusionPanel
                ekf={state?.ekf || null}
                sensors={state?.sensors || null}
                metrics={state?.metrics || null}
                truePos={{
                  x: state?.rover.x || 50,
                  y: state?.rover.y || 50,
                  speed: state?.rover.speed || 0,
                  heading_deg: state?.rover.heading_deg || 0,
                }}
              />
            </div>
          </div>
        </div>

        {/* Lower Dashboard Row: Telemetry, AI Console, Live Graphs */}
        <div className="grid grid-cols-1 md:grid-cols-3 gap-3">
          {/* Telemetry Flight Stats */}
          <div className="h-[260px]">
            <TelemetryPanel
              rover={state?.rover || null}
              metrics={state?.metrics || null}
            />
          </div>

          {/* AI Decision & Event Log */}
          <div className="h-[260px]">
            <AIDecisionConsole logs={state?.logs || []} />
          </div>

          {/* Real-time Streaming Performance Graphs */}
          <div className="h-[260px]">
            <RealtimeCharts
              metrics={state?.metrics || null}
              simTime={state?.sim_time || 0}
            />
          </div>
        </div>
      </main>

      {/* Benchmark Modal (Mode A vs B vs C) */}
      <BenchmarkModal
        isOpen={isBenchmarkModalOpen}
        onClose={() => setIsBenchmarkModalOpen(false)}
        benchmarkResult={benchmarkResult}
        isBenchmarking={isBenchmarking}
        onRunBenchmark={() => runBenchmark()}
      />
    </div>
  );
}

export default App;
