import React, { useRef, useEffect, useState, useCallback } from 'react';
import { ZoomIn, ZoomOut, Maximize2, Crosshair, AlertOctagon, Eye, Navigation, Compass } from 'lucide-react';
import { SimulationFullState } from '../types/simulation';

interface MarsCanvasProps {
  state: SimulationFullState | null;
  onInjectHazard: (x: number, y: number, radius?: number, type?: string) => void;
}

export const MarsCanvas: React.FC<MarsCanvasProps> = ({ state, onInjectHazard }) => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null);

  // Viewport Transform States (Pan & Zoom)
  const [zoom, setZoom] = useState<number>(1.0);
  const [pan, setPan] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [isDragging, setIsDragging] = useState<boolean>(false);
  const [dragStart, setDragStart] = useState<{ x: number; y: number }>({ x: 0, y: 0 });
  const [followRover, setFollowRover] = useState<boolean>(false);
  const [hoverCoord, setHoverCoord] = useState<{ x: number; y: number } | null>(null);
  const [injectMode, setInjectMode] = useState<'ROCK_FIELD' | 'CRATER' | null>(null);

  // Auto-follow rover if toggled
  useEffect(() => {
    if (followRover && state && canvasRef.current) {
      const canvas = canvasRef.current;
      const widthM = state.terrain_summary.width_m || 600;
      const heightM = state.terrain_summary.height_m || 600;
      const scale = (Math.min(canvas.width, canvas.height) / widthM) * zoom;

      const rx = state.rover.x * scale;
      const ry = state.rover.y * scale;

      setPan({
        x: canvas.width / 2 - rx,
        y: canvas.height / 2 - ry,
      });
    }
  }, [followRover, state?.rover.x, state?.rover.y, zoom]);

  // Main Canvas Rendering Loop
  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas || !state) return;
    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    // Handle high DPI display
    const dpr = window.devicePixelRatio || 1;
    const rect = canvas.getBoundingClientRect();
    if (canvas.width !== rect.width * dpr || canvas.height !== rect.height * dpr) {
      canvas.width = rect.width * dpr;
      canvas.height = rect.height * dpr;
    }

    ctx.save();
    ctx.scale(dpr, dpr);
    const viewW = rect.width;
    const viewH = rect.height;

    // Background Clear
    ctx.fillStyle = '#080d1a';
    ctx.fillRect(0, 0, viewW, viewH);

    // Apply Pan & Zoom
    const worldW = state.terrain_summary.width_m || 600;
    const worldH = state.terrain_summary.height_m || 600;
    const baseScale = Math.min(viewW, viewH) / worldW;
    const scale = baseScale * zoom;

    // Center offset
    const offsetX = pan.x + (viewW - worldW * scale) / 2;
    const offsetY = pan.y + (viewH - worldH * scale) / 2;

    const toCanvasX = (wx: number) => offsetX + wx * scale;
    const toCanvasY = (wy: number) => offsetY + wy * scale;
    const toWorldDist = (m: number) => m * scale;

    // 1. Draw Mars Surface Base
    ctx.save();
    ctx.beginPath();
    ctx.rect(toCanvasX(0), toCanvasY(0), toWorldDist(worldW), toWorldDist(worldH));
    ctx.fillStyle = '#140e0b';
    ctx.fill();
    ctx.strokeStyle = '#334155';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.clip();

    // Subtle terrain dust gradient
    const dustGrad = ctx.createRadialGradient(
      toCanvasX(worldW * 0.5),
      toCanvasY(worldH * 0.5),
      toWorldDist(50),
      toCanvasX(worldW * 0.5),
      toCanvasY(worldH * 0.5),
      toWorldDist(worldW * 0.7)
    );
    dustGrad.addColorStop(0, '#26120b');
    dustGrad.addColorStop(0.5, '#1a0d08');
    dustGrad.addColorStop(1, '#0e0806');
    ctx.fillStyle = dustGrad;
    ctx.fillRect(toCanvasX(0), toCanvasY(0), toWorldDist(worldW), toWorldDist(worldH));

    // Grid Lines (every 50 meters)
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.04)';
    ctx.lineWidth = 1;
    for (let gx = 0; gx <= worldW; gx += 50) {
      ctx.beginPath();
      ctx.moveTo(toCanvasX(gx), toCanvasY(0));
      ctx.lineTo(toCanvasX(gx), toCanvasY(worldH));
      ctx.stroke();
    }
    for (let gy = 0; gy <= worldH; gy += 50) {
      ctx.beginPath();
      ctx.moveTo(toCanvasX(0), toCanvasY(gy));
      ctx.lineTo(toCanvasX(worldW), toCanvasY(gy));
      ctx.stroke();
    }

    // 2. Draw Natural Rock Fields
    if (state.terrain_summary.rock_fields) {
      for (const rf of state.terrain_summary.rock_fields) {
        ctx.save();
        const cx = toCanvasX(rf.x);
        const cy = toCanvasY(rf.y);
        const rad = toWorldDist(rf.radius);

        const rockGrad = ctx.createRadialGradient(cx, cy, 2, cx, cy, rad);
        rockGrad.addColorStop(0, 'rgba(180, 83, 9, 0.25)');
        rockGrad.addColorStop(0.8, 'rgba(120, 53, 15, 0.12)');
        rockGrad.addColorStop(1, 'rgba(120, 53, 15, 0)');
        ctx.fillStyle = rockGrad;
        ctx.beginPath();
        ctx.arc(cx, cy, rad, 0, Math.PI * 2);
        ctx.fill();

        // Small scattered rock markers
        ctx.fillStyle = '#b45309';
        for (let i = 0; i < 5; i++) {
          const angle = (i * Math.PI * 2) / 5;
          const rSub = rad * 0.55;
          ctx.beginPath();
          ctx.arc(cx + Math.cos(angle) * rSub, cy + Math.sin(angle) * rSub, 2.5, 0, Math.PI * 2);
          ctx.fill();
        }
        ctx.restore();
      }
    }

    // 3. Draw Craters
    if (state.terrain_summary.craters) {
      for (const cr of state.terrain_summary.craters) {
        ctx.save();
        const cx = toCanvasX(cr.x);
        const cy = toCanvasY(cr.y);
        const rad = toWorldDist(cr.radius);

        // Crater depression
        const crGrad = ctx.createRadialGradient(cx, cy, rad * 0.1, cx, cy, rad);
        crGrad.addColorStop(0, '#050302');
        crGrad.addColorStop(0.7, '#1a0b06');
        crGrad.addColorStop(1, '#3b170c');
        ctx.fillStyle = crGrad;
        ctx.beginPath();
        ctx.arc(cx, cy, rad, 0, Math.PI * 2);
        ctx.fill();

        // Crater Raised Rim
        ctx.strokeStyle = '#7c2d12';
        ctx.lineWidth = 2;
        ctx.beginPath();
        ctx.arc(cx, cy, rad, 0, Math.PI * 2);
        ctx.stroke();

        // Inner steep slope warning ring
        ctx.strokeStyle = 'rgba(239, 68, 68, 0.25)';
        ctx.setLineDash([3, 3]);
        ctx.beginPath();
        ctx.arc(cx, cy, rad * 0.85, 0, Math.PI * 2);
        ctx.stroke();
        ctx.restore();
      }
    }

    // 4. Draw Injected / Dynamic Hazards (Alert Red Rings)
    if (state.terrain_summary.injected_hazards) {
      for (const hz of state.terrain_summary.injected_hazards) {
        ctx.save();
        const hx = toCanvasX(hz.x);
        const hy = toCanvasY(hz.y);
        const hrad = toWorldDist(hz.radius);

        // Danger fill
        const hzGrad = ctx.createRadialGradient(hx, hy, 2, hx, hy, hrad);
        hzGrad.addColorStop(0, 'rgba(239, 68, 68, 0.45)');
        hzGrad.addColorStop(0.7, 'rgba(239, 68, 68, 0.2)');
        hzGrad.addColorStop(1, 'rgba(239, 68, 68, 0)');
        ctx.fillStyle = hzGrad;
        ctx.beginPath();
        ctx.arc(hx, hy, hrad, 0, Math.PI * 2);
        ctx.fill();

        // Pulsing warning border
        ctx.strokeStyle = '#ef4444';
        ctx.lineWidth = 2;
        ctx.setLineDash([4, 4]);
        ctx.beginPath();
        ctx.arc(hx, hy, hrad, 0, Math.PI * 2);
        ctx.stroke();

        // Hazard Label
        ctx.fillStyle = '#fca5a5';
        ctx.font = '9px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(`⚠️ ${hz.type || 'HAZARD'}`, hx, hy - hrad - 4);
        ctx.restore();
      }
    }

    // 5. Draw Start Point & Science Target
    const startPos = state.terrain_summary.start_pos || [50, 50];
    const targetPos = state.terrain_summary.target_pos || [540, 530];

    // Start Marker
    ctx.save();
    const sx = toCanvasX(startPos[0]);
    const sy = toCanvasY(startPos[1]);
    ctx.fillStyle = 'rgba(59, 130, 246, 0.2)';
    ctx.beginPath();
    ctx.arc(sx, sy, 12, 0, Math.PI * 2);
    ctx.fill();
    ctx.strokeStyle = '#3b82f6';
    ctx.lineWidth = 1.5;
    ctx.stroke();
    ctx.fillStyle = '#60a5fa';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('START (ORIGIN)', sx, sy + 22);
    ctx.restore();

    // Science Target Beacon (Pulsing Green)
    ctx.save();
    const tx = toCanvasX(targetPos[0]);
    const ty = toCanvasY(targetPos[1]);
    const pulseRad = 16 + Math.sin(Date.now() / 250) * 4;

    ctx.strokeStyle = 'rgba(16, 185, 129, 0.4)';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.arc(tx, ty, pulseRad, 0, Math.PI * 2);
    ctx.stroke();

    ctx.fillStyle = '#10b981';
    ctx.beginPath();
    ctx.arc(tx, ty, 6, 0, Math.PI * 2);
    ctx.fill();

    // Reticle crosshair
    ctx.strokeStyle = '#10b981';
    ctx.lineWidth = 1.5;
    ctx.beginPath();
    ctx.moveTo(tx - 12, ty);
    ctx.lineTo(tx + 12, ty);
    ctx.moveTo(tx, ty - 12);
    ctx.lineTo(tx, ty + 12);
    ctx.stroke();

    ctx.fillStyle = '#34d399';
    ctx.font = '10px "JetBrains Mono", monospace';
    ctx.textAlign = 'center';
    ctx.fillText('🎯 SCIENCE TARGET', tx, ty - 18);
    ctx.restore();

    // 6. Draw Trajectories
    // 6a. Historical Breadcrumb Driven Path
    if (state.routes.breadcrumb_history && state.routes.breadcrumb_history.length > 1) {
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(toCanvasX(state.routes.breadcrumb_history[0].x), toCanvasY(state.routes.breadcrumb_history[0].y));
      for (let i = 1; i < state.routes.breadcrumb_history.length; i++) {
        ctx.lineTo(toCanvasX(state.routes.breadcrumb_history[i].x), toCanvasY(state.routes.breadcrumb_history[i].y));
      }
      ctx.strokeStyle = '#0284c7';
      ctx.lineWidth = 2.5;
      ctx.stroke();
      ctx.restore();
    }

    // 6b. Old Avoided Route (Dashed Red with X markers)
    if (state.routes.old_avoided_path && state.routes.old_avoided_path.length > 1) {
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(toCanvasX(state.routes.old_avoided_path[0][0]), toCanvasY(state.routes.old_avoided_path[0][1]));
      for (let i = 1; i < state.routes.old_avoided_path.length; i++) {
        ctx.lineTo(toCanvasX(state.routes.old_avoided_path[i][0]), toCanvasY(state.routes.old_avoided_path[i][1]));
      }
      ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)';
      ctx.lineWidth = 2;
      ctx.setLineDash([5, 5]);
      ctx.stroke();

      // Red X marks along old path
      ctx.fillStyle = '#ef4444';
      ctx.font = '10px sans-serif';
      ctx.textAlign = 'center';
      for (let i = 1; i < state.routes.old_avoided_path.length - 1; i += 2) {
        const px = toCanvasX(state.routes.old_avoided_path[i][0]);
        const py = toCanvasY(state.routes.old_avoided_path[i][1]);
        ctx.fillText('❌', px, py + 4);
      }
      ctx.restore();
    }

    // 6c. Active Planned Safe Route (Glowing Cyan Line)
    if (state.routes.current_planned && state.routes.current_planned.length > 1) {
      ctx.save();
      ctx.beginPath();
      ctx.moveTo(toCanvasX(state.routes.current_planned[0][0]), toCanvasY(state.routes.current_planned[0][1]));
      for (let i = 1; i < state.routes.current_planned.length; i++) {
        ctx.lineTo(toCanvasX(state.routes.current_planned[i][0]), toCanvasY(state.routes.current_planned[i][1]));
      }
      ctx.strokeStyle = '#00f0ff';
      ctx.lineWidth = 2.5;
      ctx.shadowColor = '#00f0ff';
      ctx.shadowBlur = 8;
      ctx.stroke();

      // Waypoint Dots
      ctx.fillStyle = '#ffffff';
      for (const pt of state.routes.current_planned) {
        ctx.beginPath();
        ctx.arc(toCanvasX(pt[0]), toCanvasY(pt[1]), 3, 0, Math.PI * 2);
        ctx.fill();
      }
      ctx.restore();
    }

    // 6d. AI Predicted Trajectory (Amber Glowing Horizon Curve)
    if (
      state.trajectory_prediction &&
      state.trajectory_prediction.predicted_trajectory &&
      state.trajectory_prediction.predicted_trajectory.length > 1
    ) {
      const pred = state.trajectory_prediction.predicted_trajectory;
      ctx.save();

      // Lateral variance corridor
      ctx.beginPath();
      for (let i = 0; i < pred.length; i++) {
        const lateralVar = toWorldDist(1.5 + (i * 0.15));
        const px = toCanvasX(pred[i].x);
        const py = toCanvasY(pred[i].y);
        if (i === 0) ctx.moveTo(px + lateralVar, py);
        else ctx.lineTo(px + lateralVar, py);
      }
      for (let i = pred.length - 1; i >= 0; i--) {
        const lateralVar = toWorldDist(1.5 + (i * 0.15));
        const px = toCanvasX(pred[i].x);
        const py = toCanvasY(pred[i].y);
        ctx.lineTo(px - lateralVar, py);
      }
      ctx.closePath();
      ctx.fillStyle = 'rgba(245, 158, 11, 0.12)';
      ctx.fill();

      // Predicted centerline
      ctx.beginPath();
      ctx.moveTo(toCanvasX(pred[0].x), toCanvasY(pred[0].y));
      for (let i = 1; i < pred.length; i++) {
        ctx.lineTo(toCanvasX(pred[i].x), toCanvasY(pred[i].y));
      }
      ctx.strokeStyle = '#f59e0b';
      ctx.lineWidth = 2.5;
      ctx.shadowColor = '#f59e0b';
      ctx.shadowBlur = 6;
      ctx.stroke();

      // End of prediction horizon flag
      const lastPt = pred[pred.length - 1];
      ctx.fillStyle = '#fbbf24';
      ctx.font = '9px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`+30s AI PROJECTION (${state.trajectory_prediction.confidence_pct}%)`, toCanvasX(lastPt.x), toCanvasY(lastPt.y) - 6);
      ctx.restore();
    }

    // 7. Draw Rover, FOV Camera Cone, and EKF Uncertainty Ellipse
    const rx = toCanvasX(state.rover.x);
    const ry = toCanvasY(state.rover.y);
    const heading = state.rover.heading_rad;

    // 7a. Camera Forward FOV Cone
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(heading);
    const fovLen = toWorldDist(45);
    const fovAngle = Math.PI * 0.38; // ~70 deg FOV

    const fovGrad = ctx.createRadialGradient(0, 0, 2, 0, 0, fovLen);
    fovGrad.addColorStop(0, 'rgba(0, 240, 255, 0.25)');
    fovGrad.addColorStop(0.8, 'rgba(0, 240, 255, 0.06)');
    fovGrad.addColorStop(1, 'rgba(0, 240, 255, 0)');

    ctx.beginPath();
    ctx.moveTo(0, 0);
    ctx.arc(0, 0, fovLen, -fovAngle / 2, fovAngle / 2);
    ctx.closePath();
    ctx.fillStyle = fovGrad;
    ctx.fill();
    ctx.strokeStyle = 'rgba(0, 240, 255, 0.3)';
    ctx.lineWidth = 1;
    ctx.stroke();
    ctx.restore();

    // 7b. EKF Covariance Uncertainty Ellipse (2-sigma)
    if (state.ekf && state.ekf.covariance_ellipse) {
      const ell = state.ekf.covariance_ellipse;
      const semiMajor = toWorldDist(ell.semi_major_m);
      const semiMinor = toWorldDist(ell.semi_minor_m);
      const angle = ell.angle_rad;

      ctx.save();
      ctx.translate(toCanvasX(state.ekf.est_x), toCanvasY(state.ekf.est_y));
      ctx.rotate(angle);

      ctx.beginPath();
      ctx.ellipse(0, 0, Math.max(3, semiMajor), Math.max(3, semiMinor), 0, 0, Math.PI * 2);
      ctx.fillStyle = state.ekf.fallback_active ? 'rgba(239, 68, 68, 0.2)' : 'rgba(0, 240, 255, 0.15)';
      ctx.fill();

      ctx.strokeStyle = state.ekf.fallback_active ? '#ef4444' : '#00f0ff';
      ctx.lineWidth = 1.5;
      ctx.setLineDash([3, 3]);
      ctx.stroke();

      // Uncertainty meter label
      ctx.fillStyle = state.ekf.fallback_active ? '#fca5a5' : '#a5f3fc';
      ctx.font = '9px "JetBrains Mono", monospace';
      ctx.textAlign = 'center';
      ctx.fillText(`2σ: ±${state.ekf.uncertainty_m}m`, 0, -semiMinor - 4);
      ctx.restore();
    }

    // 7c. Rover Physical Icon & Chassis
    ctx.save();
    ctx.translate(rx, ry);
    ctx.rotate(heading);

    // Chassis Box
    ctx.fillStyle = '#1e293b';
    ctx.fillRect(-10, -7, 20, 14);
    ctx.strokeStyle = '#00f0ff';
    ctx.lineWidth = 1.5;
    ctx.strokeRect(-10, -7, 20, 14);

    // Wheels (6 wheels)
    ctx.fillStyle = '#0f172a';
    ctx.strokeStyle = '#64748b';
    ctx.lineWidth = 1;
    const wheelPositions = [
      [-8, -9], [0, -9], [8, -9],
      [-8, 7], [0, 7], [8, 7],
    ];
    for (const [wx, wy] of wheelPositions) {
      ctx.fillRect(wx - 3, wy, 6, 2.5);
      ctx.strokeRect(wx - 3, wy, 6, 2.5);
    }

    // Direction Mast / Pointer Arrow
    ctx.fillStyle = '#ef4444';
    ctx.beginPath();
    ctx.moveTo(12, 0);
    ctx.lineTo(6, -4);
    ctx.lineTo(6, 4);
    ctx.closePath();
    ctx.fill();

    // Center Core Light
    ctx.fillStyle = state.ekf.fallback_active ? '#ef4444' : '#10b981';
    ctx.beginPath();
    ctx.arc(0, 0, 3, 0, Math.PI * 2);
    ctx.fill();

    ctx.restore();

    // 8. Draw Satellite Constellation & Beams
    if (state.satellites) {
      for (const sat of state.satellites) {
        const satX = toCanvasX(sat.gx);
        const satY = toCanvasY(sat.gy);
        const fovRadius = toWorldDist(sat.fov_radius_m);

        // Draw Ground Footprint / FOV Cone
        ctx.save();
        ctx.beginPath();
        ctx.arc(satX, satY, fovRadius, 0, Math.PI * 2);
        ctx.strokeStyle = sat.is_visible
          ? (sat.current_task === 'PNT' ? 'rgba(0, 240, 255, 0.25)' : sat.current_task === 'IMAGING' ? 'rgba(168, 85, 247, 0.25)' : 'rgba(16, 185, 129, 0.25)')
          : 'rgba(255, 255, 255, 0.05)';
        ctx.lineWidth = 1;
        ctx.setLineDash([4, 4]);
        ctx.stroke();

        if (sat.is_visible && sat.current_task !== 'IDLE') {
          ctx.fillStyle = sat.current_task === 'PNT' ? 'rgba(0, 240, 255, 0.04)' : sat.current_task === 'IMAGING' ? 'rgba(168, 85, 247, 0.04)' : 'rgba(16, 185, 129, 0.04)';
          ctx.fill();
        }
        ctx.restore();

        // Draw Active Beam Link from Satellite to Rover
        if (sat.is_visible && !sat.is_outage && sat.current_task !== 'IDLE') {
          ctx.save();
          ctx.beginPath();
          ctx.moveTo(satX, satY);
          ctx.lineTo(rx, ry);

          if (sat.current_task === 'PNT') {
            ctx.strokeStyle = 'rgba(0, 240, 255, 0.85)';
            ctx.shadowColor = '#00f0ff';
            ctx.shadowBlur = 10;
            ctx.lineWidth = 2;
          } else if (sat.current_task === 'IMAGING') {
            ctx.strokeStyle = 'rgba(168, 85, 247, 0.85)';
            ctx.shadowColor = '#a855f7';
            ctx.shadowBlur = 10;
            ctx.lineWidth = 2;
          } else {
            ctx.strokeStyle = 'rgba(16, 185, 129, 0.85)';
            ctx.shadowColor = '#10b981';
            ctx.shadowBlur = 8;
            ctx.lineWidth = 1.5;
            ctx.setLineDash([6, 6]);
          }
          ctx.stroke();
          ctx.restore();
        }

        // Outage Beam (Flashing Red if outage forced)
        if (sat.is_outage) {
          ctx.save();
          ctx.beginPath();
          ctx.moveTo(satX, satY);
          ctx.lineTo(rx, ry);
          ctx.strokeStyle = 'rgba(239, 68, 68, 0.6)';
          ctx.lineWidth = 1.5;
          ctx.setLineDash([4, 6]);
          ctx.stroke();
          ctx.restore();
        }

        // Draw Satellite Icon / Marker
        ctx.save();
        ctx.translate(satX, satY);

        // Satellite Solar Array panels
        ctx.fillStyle = '#1e3a8a';
        ctx.fillRect(-14, -3, 8, 6);
        ctx.fillRect(6, -3, 8, 6);
        ctx.strokeStyle = '#60a5fa';
        ctx.lineWidth = 1;
        ctx.strokeRect(-14, -3, 8, 6);
        ctx.strokeRect(6, -3, 8, 6);

        // Satellite Body
        const satColor = sat.is_outage
          ? '#ef4444'
          : sat.is_visible
          ? (sat.current_task === 'PNT' ? '#00f0ff' : sat.current_task === 'IMAGING' ? '#a855f7' : '#10b981')
          : '#64748b';

        ctx.fillStyle = '#0f172a';
        ctx.fillRect(-5, -5, 10, 10);
        ctx.strokeStyle = satColor;
        ctx.lineWidth = 1.5;
        ctx.strokeRect(-5, -5, 10, 10);

        // Center beacon
        ctx.fillStyle = satColor;
        ctx.beginPath();
        ctx.arc(0, 0, 2.5, 0, Math.PI * 2);
        ctx.fill();

        // Sat Label & Task
        ctx.fillStyle = satColor;
        ctx.font = 'bold 9px "JetBrains Mono", monospace';
        ctx.textAlign = 'center';
        ctx.fillText(sat.sat_id, 0, -10);

        ctx.fillStyle = '#94a3b8';
        ctx.font = '8px "JetBrains Mono", monospace';
        ctx.fillText(sat.current_task, 0, 16);

        ctx.restore();
      }
    }

    // 9. Coordinate Hover HUD
    if (hoverCoord) {
      const hx = toCanvasX(hoverCoord.x);
      const hy = toCanvasY(hoverCoord.y);
      ctx.save();
      ctx.strokeStyle = 'rgba(255, 255, 255, 0.4)';
      ctx.lineWidth = 1;
      ctx.setLineDash([2, 2]);
      ctx.strokeRect(hx - 8, hy - 8, 16, 16);
      ctx.restore();
    }

    ctx.restore(); // Restore clip
    ctx.restore(); // Restore dpr
  }, [state, zoom, pan, hoverCoord]);

  // Pan & Zoom Event Handlers
  const handleMouseDown = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (injectMode && state && canvasRef.current) {
      // Inject hazard at clicked position
      const rect = canvasRef.current.getBoundingClientRect();
      const clickX = e.clientX - rect.left;
      const clickY = e.clientY - rect.top;

      const worldW = state.terrain_summary.width_m || 600;
      const worldH = state.terrain_summary.height_m || 600;
      const baseScale = Math.min(rect.width, rect.height) / worldW;
      const scale = baseScale * zoom;
      const offsetX = pan.x + (rect.width - worldW * scale) / 2;
      const offsetY = pan.y + (rect.height - worldH * scale) / 2;

      const worldX = (clickX - offsetX) / scale;
      const worldY = (clickY - offsetY) / scale;

      onInjectHazard(worldX, worldY, injectMode === 'CRATER' ? 35.0 : 28.0, injectMode);
      setInjectMode(null);
      return;
    }

    setIsDragging(true);
    setDragStart({ x: e.clientX - pan.x, y: e.clientY - pan.y });
    setFollowRover(false);
  };

  const handleMouseMove = (e: React.MouseEvent<HTMLCanvasElement>) => {
    if (isDragging) {
      setPan({
        x: e.clientX - dragStart.x,
        y: e.clientY - dragStart.y,
      });
    }

    if (state && canvasRef.current) {
      const rect = canvasRef.current.getBoundingClientRect();
      const mouseX = e.clientX - rect.left;
      const mouseY = e.clientY - rect.top;

      const worldW = state.terrain_summary.width_m || 600;
      const worldH = state.terrain_summary.height_m || 600;
      const baseScale = Math.min(rect.width, rect.height) / worldW;
      const scale = baseScale * zoom;
      const offsetX = pan.x + (rect.width - worldW * scale) / 2;
      const offsetY = pan.y + (rect.height - worldH * scale) / 2;

      const wx = (mouseX - offsetX) / scale;
      const wy = (mouseY - offsetY) / scale;

      if (wx >= 0 && wx <= worldW && wy >= 0 && wy <= worldH) {
        setHoverCoord({ x: Math.round(wx), y: Math.round(wy) });
      } else {
        setHoverCoord(null);
      }
    }
  };

  const handleMouseUp = () => {
    setIsDragging(false);
  };

  const handleWheel = (e: React.WheelEvent<HTMLCanvasElement>) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    setZoom((prev) => Math.max(0.6, Math.min(4.0, prev * zoomFactor)));
  };

  const resetView = () => {
    setZoom(1.0);
    setPan({ x: 0, y: 0 });
    setFollowRover(false);
  };

  return (
    <div className="relative w-full h-full min-h-[460px] flex flex-col panel-glass overflow-hidden">
      {/* Canvas Header Bar */}
      <div className="panel-glass-header">
        <div className="flex items-center gap-2">
          <Crosshair className="w-4 h-4 text-neon-cyan" />
          <span className="font-bold text-slate-200">MARS TACTICAL SURFACE & ORBITAL RADAR</span>
          <span className="text-[10px] text-slate-500 font-mono">600m × 600m COORD GRID</span>
        </div>
        <div className="flex items-center gap-3">
          {hoverCoord && (
            <div className="hidden sm:flex items-center gap-2 font-mono text-[11px] text-neon-cyan bg-space-850 px-2 py-0.5 rounded border border-slate-800">
              <span>X: {hoverCoord.x}m</span>
              <span>Y: {hoverCoord.y}m</span>
            </div>
          )}
          <div className="flex items-center gap-1 text-[10px] font-mono text-slate-400">
            <span>ZOOM: {(zoom * 100).toFixed(0)}%</span>
          </div>
        </div>
      </div>

      {/* Main HTML5 Canvas Area */}
      <div className="relative flex-1 w-full h-full bg-space-950 cursor-crosshair">
        <canvas
          ref={canvasRef}
          onMouseDown={handleMouseDown}
          onMouseMove={handleMouseMove}
          onMouseUp={handleMouseUp}
          onMouseLeave={handleMouseUp}
          onWheel={handleWheel}
          className="w-full h-full block"
        />

        {/* Tactical Overlay Legend */}
        <div className="absolute top-3 left-3 flex flex-col gap-1.5 p-2 rounded-lg bg-space-900/85 backdrop-blur-md border border-slate-800 text-[10px] font-mono text-slate-300 pointer-events-none select-none shadow-lg">
          <div className="flex items-center gap-2">
            <span className="w-3 h-0.5 bg-neon-cyan inline-block shadow-neon-cyan" />
            <span>Planned Safe Route</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="w-3 h-0.5 bg-amber-400 inline-block shadow-neon-amber" />
            <span>AI Trajectory (+30s)</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="w-3 h-0.5 border-b border-dashed border-red-500 inline-block" />
            <span>Old Avoided Route (❌)</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="w-2.5 h-2.5 rounded-full border border-dashed border-neon-cyan inline-block" />
            <span>EKF Uncertainty (2σ)</span>
          </div>
          <div className="flex items-center gap-2">
            <span className="w-2.5 h-2.5 rounded-full bg-neon-emerald inline-block" />
            <span>Science Target</span>
          </div>
        </div>

        {/* Canvas Toolbar Floating Controls */}
        <div className="absolute top-3 right-3 flex items-center gap-1.5 p-1 rounded-lg bg-space-900/85 backdrop-blur-md border border-slate-800 shadow-lg">
          <button
            onClick={() => setZoom((z) => Math.min(4.0, z * 1.2))}
            className="p-1.5 rounded hover:bg-slate-800 text-slate-300 hover:text-white transition-colors"
            title="Zoom In"
          >
            <ZoomIn className="w-4 h-4" />
          </button>
          <button
            onClick={() => setZoom((z) => Math.max(0.6, z * 0.8))}
            className="p-1.5 rounded hover:bg-slate-800 text-slate-300 hover:text-white transition-colors"
            title="Zoom Out"
          >
            <ZoomOut className="w-4 h-4" />
          </button>
          <button
            onClick={resetView}
            className="p-1.5 rounded hover:bg-slate-800 text-slate-300 hover:text-white transition-colors"
            title="Reset View"
          >
            <Maximize2 className="w-4 h-4" />
          </button>
          <button
            onClick={() => setFollowRover((f) => !f)}
            className={`p-1.5 rounded transition-colors ${followRover ? 'bg-neon-cyan/20 text-neon-cyan border border-neon-cyan/30' : 'hover:bg-slate-800 text-slate-300'}`}
            title="Follow Rover"
          >
            <Navigation className="w-4 h-4" />
          </button>
        </div>

        {/* Hazard Placement Tooltips */}
        {injectMode && (
          <div className="absolute bottom-4 left-1/2 -translate-x-1/2 px-4 py-2 rounded-xl bg-neon-crimson/20 border border-neon-crimson text-neon-crimson font-mono text-xs shadow-neon-crimson animate-pulse flex items-center gap-2">
            <AlertOctagon className="w-4 h-4" />
            <span>CLICK ANYWHERE ON MAP TO INJECT {injectMode}</span>
            <button
              onClick={() => setInjectMode(null)}
              className="ml-2 px-1.5 py-0.5 rounded bg-slate-800 text-white text-[10px]"
            >
              CANCEL
            </button>
          </div>
        )}

        {/* Bottom Quick Action: Click-to-inject shortcut buttons */}
        <div className="absolute bottom-3 right-3 flex items-center gap-2">
          <button
            onClick={() => setInjectMode(injectMode === 'ROCK_FIELD' ? null : 'ROCK_FIELD')}
            className={`btn-aerospace border text-[11px] ${injectMode === 'ROCK_FIELD' ? 'bg-neon-crimson text-white border-neon-crimson' : 'bg-space-900/80 text-amber-300 border-amber-500/40 hover:bg-space-850'}`}
          >
            <AlertOctagon className="w-3.5 h-3.5" />
            <span>{injectMode === 'ROCK_FIELD' ? 'CANCEL CLICK' : '+ INJECT ROCK HAZARD'}</span>
          </button>
        </div>
      </div>
    </div>
  );
};
