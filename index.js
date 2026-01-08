
import * as m2d from './matrix2d.js';
import { sub2, dot2, perp2, lerp2, distance2, direction2, ray2, scale2 } from './geometry2d.js';
import { add2a, scale2a } from './geometry2a.js';
import { churn, fold, random } from './xorshift128.js';

const TAU = Math.PI * 2;

const seed = [0xb0b5c0ff, 0xeefacade, Math.random() * 0xffffff, Math.random() * 0xffffff];

const project = ({x, y}, C, R, Z) => {
  const angle = Math.atan2(y, x);
  const hypot = Math.hypot(x, y);
  const radius = Math.atan2(hypot, Z) / TAU * 4 * R;
  return ray2(C, angle, radius);
};

const drawLine = (plotter, source, target, C, R, Z, T) => {
  const points = [source, target];
  let meter = 1000;
  while (points.length >= 2 && meter-- > 0) {
    const c = points.pop();
    const a = points.at(-1);
    const ap = project(a, C, R, Z);
    const cp = project(c, C, R, Z);
    if (distance2(ap, cp) > T) {
      const b = lerp2(a, c, 0.5);
      points.push(b);
      points.push(c);
    } else {
      plotter.beginPath();
      plotter.moveTo(ap.x, ap.y);
      plotter.lineTo(cp.x, cp.y);
      plotter.stroke();
    }
  }
};

// Draw visual HUD elements
const drawHUD = (plotter, viewportSize, hudData) => {
  const margin = 20;
  const hudWidth = 150;
  const hudHeight = 200;
  const left = margin;
  const top = margin;

  plotter.save();

  // HUD background
  plotter.fillStyle = 'rgba(0, 0, 0, 0.5)';
  plotter.fillRect(left, top, hudWidth, hudHeight);
  plotter.strokeStyle = '#444';
  plotter.strokeRect(left, top, hudWidth, hudHeight);

  // Range indicator (vertical bar)
  const rangeBarX = left + 15;
  const rangeBarY = top + 30;
  const rangeBarHeight = 100;
  const rangeBarWidth = 12;

  plotter.strokeStyle = '#666';
  plotter.strokeRect(rangeBarX, rangeBarY, rangeBarWidth, rangeBarHeight);

  // Range fill (clamped 0-2000m mapped to bar)
  const rangeNorm = Math.max(0, Math.min(1, hudData.range / 2000));
  const rangeFillHeight = rangeBarHeight * (1 - rangeNorm);
  plotter.fillStyle = rangeNorm < 0.1 ? '#f00' : rangeNorm < 0.3 ? '#ff0' : '#0f0';
  plotter.fillRect(rangeBarX + 1, rangeBarY + rangeBarHeight - rangeFillHeight, rangeBarWidth - 2, rangeFillHeight);

  plotter.fillStyle = '#fff';
  plotter.font = '10px monospace';
  plotter.fillText('RNG', rangeBarX - 2, rangeBarY - 5);
  plotter.fillText(`${Math.floor(hudData.range)}m`, rangeBarX - 5, rangeBarY + rangeBarHeight + 15);

  // Speed indicator (horizontal bar)
  const speedBarX = left + 40;
  const speedBarY = top + 30;
  const speedBarWidth = 80;
  const speedBarHeight = 12;

  plotter.strokeStyle = '#666';
  plotter.strokeRect(speedBarX, speedBarY, speedBarWidth, speedBarHeight);

  // Speed fill (clamped to ±1 m/s)
  const speedNorm = Math.max(-1, Math.min(1, hudData.rangeSpeed));
  const speedMid = speedBarX + speedBarWidth / 2;
  if (speedNorm > 0) {
    plotter.fillStyle = '#f80';
    plotter.fillRect(speedMid, speedBarY + 1, (speedBarWidth / 2) * speedNorm, speedBarHeight - 2);
  } else {
    plotter.fillStyle = '#08f';
    plotter.fillRect(speedMid + (speedBarWidth / 2) * speedNorm, speedBarY + 1, -(speedBarWidth / 2) * speedNorm, speedBarHeight - 2);
  }

  // Center line
  plotter.strokeStyle = '#fff';
  plotter.beginPath();
  plotter.moveTo(speedMid, speedBarY);
  plotter.lineTo(speedMid, speedBarY + speedBarHeight);
  plotter.stroke();

  plotter.fillStyle = '#fff';
  plotter.fillText('SPD', speedBarX + speedBarWidth / 2 - 10, speedBarY - 5);

  // Heading indicator (compass)
  const compassX = left + 80;
  const compassY = top + 100;
  const compassRadius = 30;

  plotter.strokeStyle = '#666';
  plotter.beginPath();
  plotter.arc(compassX, compassY, compassRadius, 0, TAU);
  plotter.stroke();

  // Heading needle
  const headingAngle = hudData.heading * TAU - TAU / 4; // Convert to radians, adjust for display
  plotter.strokeStyle = '#0f0';
  plotter.lineWidth = 2;
  plotter.beginPath();
  plotter.moveTo(compassX, compassY);
  plotter.lineTo(
    compassX + Math.cos(headingAngle) * compassRadius * 0.8,
    compassY + Math.sin(headingAngle) * compassRadius * 0.8
  );
  plotter.stroke();
  plotter.lineWidth = 1;

  // Target direction indicator (if not heading towards it)
  plotter.strokeStyle = '#f00';
  plotter.beginPath();
  plotter.arc(compassX, compassY, compassRadius - 5, -TAU / 4 - 0.1, -TAU / 4 + 0.1);
  plotter.stroke();

  plotter.fillStyle = '#fff';
  plotter.fillText('HDG', compassX - 10, compassY - compassRadius - 5);

  // Anchor status
  const anchorX = left + 15;
  const anchorY = top + 160;
  plotter.fillStyle = hudData.anchored ? '#0ff' : '#444';
  plotter.beginPath();
  plotter.arc(anchorX + 8, anchorY + 8, 8, 0, TAU);
  plotter.fill();
  plotter.fillStyle = '#fff';
  plotter.fillText('ANCHOR', anchorX + 22, anchorY + 12);

  // Spin indicator
  plotter.fillStyle = '#fff';
  plotter.fillText(`SPIN: ${(hudData.spin * 1000).toFixed(2)}τ/s`, left + 10, top + hudHeight - 10);

  // Modeline at bottom of screen
  const modeline = 'W/↑:forward  S/↓:back  A/←:port  D/→:stbd  Q:rotate←  E:rotate→  SPACE:anchor';
  plotter.font = '12px monospace';
  plotter.fillStyle = 'rgba(0, 0, 0, 0.5)';
  const modelineWidth = plotter.measureText(modeline).width + 20;
  const modelineX = (viewportSize.x - modelineWidth) / 2;
  const modelineY = viewportSize.y - margin;
  plotter.fillRect(modelineX, modelineY - 16, modelineWidth, 22);
  plotter.fillStyle = '#fff';
  plotter.fillText(modeline, modelineX + 10, modelineY);

  plotter.restore();
};

const drawAbstractVessel = (plotter, center, direction, radius, C, R, Z, T) => {
  const top = ray2(center, direction, radius);
  const bottom = ray2(center, direction + TAU/2, radius);
  const projectedTop = project(top, C, R, Z);
  const projectedBottom = project(bottom, C, R, Z);
  if (distance2(projectedTop, projectedBottom) >= T) {
    return { top };
  }
  const angle = Math.atan2(projectedTop.y - projectedBottom.y, projectedTop.x - projectedBottom.x);
  const projectedCenter = project(center, C, R, Z);
  const t = ray2(projectedCenter, angle, T/2);
  const b = ray2(projectedCenter, angle+TAU/2, T/2);
  const p = ray2(b, angle+TAU/4, T/2);
  const s = ray2(b, angle-TAU/4, T/2);
  plotter.beginPath();
  plotter.moveTo(t.x, t.y);
  plotter.lineTo(b.x, b.y);
  plotter.stroke();
  plotter.moveTo(p.x, p.y);
  plotter.lineTo(s.x, s.y);
  plotter.stroke();
  return null;
};

const drawVessel = (plotter, center, direction, radius, C, R, Z, T, thrustState = null) => {
  const concrete = drawAbstractVessel(plotter, center, direction, radius, C, R, Z, T);
  if (concrete === null) {
    return;
  }
  const { top } = concrete;
  const port = ray2(center, direction + TAU * 2/5, radius);
  const stbd = ray2(center, direction + TAU * 3/5, radius);
  drawLine(plotter, top, port, C, R, Z, T);
  drawLine(plotter, top, stbd, C, R, Z, T);
  drawLine(plotter, port, stbd, C, R, Z, T);

  // Draw thruster flames if thrust state is provided
  if (thrustState) {
    const flameLength = radius * 1.5;

    // Main thruster (W key) - flame out the back
    if (thrustState.forward) {
      plotter.strokeStyle = '#f80';
      const thrustPoint = ray2(center, direction + TAU/2, radius);
      const flameEnd = ray2(center, direction + TAU/2, radius + flameLength);
      drawLine(plotter, thrustPoint, flameEnd, C, R, Z, T);
    }

    // Reverse thruster (S key) - flame out the front
    if (thrustState.backward) {
      plotter.strokeStyle = '#f80';
      const flameEnd = ray2(center, direction, radius + flameLength);
      drawLine(plotter, top, flameEnd, C, R, Z, T);
    }

    const lateralFlame = flameLength * 0.5;
    // Outward directions for each corner (radially away from center)
    const portOutward = direction + TAU * 2/5;
    const stbdOutward = direction + TAU * 3/5;
    // Bow-side thruster positions (partway up each edge toward bow)
    const portBow = lerp2(port, top, 0.7);
    const stbdBow = lerp2(stbd, top, 0.7);

    // Lateral thrusters (A/D keys) - single jet fires outward from opposite side
    // Port strafe (A key) - fire from stbd corner outward
    if (thrustState.left) {
      plotter.strokeStyle = '#f80';
      const flame = ray2(stbd, stbdOutward, lateralFlame);
      drawLine(plotter, stbd, flame, C, R, Z, T);
    }

    // Starboard strafe (D key) - fire from port corner outward
    if (thrustState.right) {
      plotter.strokeStyle = '#f80';
      const flame = ray2(port, portOutward, lateralFlame);
      drawLine(plotter, port, flame, C, R, Z, T);
    }

    // Rotation thrusters (Q/E keys) - paired thrusters firing outward
    // CCW (Q): stbd-bow fires stbd-outward + port-stern fires port-outward
    if (thrustState.rotateLeft) {
      plotter.strokeStyle = '#f80';
      const stbdBowFlame = ray2(stbdBow, stbdOutward, lateralFlame);
      drawLine(plotter, stbdBow, stbdBowFlame, C, R, Z, T);
      const portSternFlame = ray2(port, portOutward, lateralFlame);
      drawLine(plotter, port, portSternFlame, C, R, Z, T);
    }

    // CW (E): port-bow fires port-outward + stbd-stern fires stbd-outward
    if (thrustState.rotateRight) {
      plotter.strokeStyle = '#f80';
      const portBowFlame = ray2(portBow, portOutward, lateralFlame);
      drawLine(plotter, portBow, portBowFlame, C, R, Z, T);
      const stbdSternFlame = ray2(stbd, stbdOutward, lateralFlame);
      drawLine(plotter, stbd, stbdSternFlame, C, R, Z, T);
    }

    // Reset stroke style
    plotter.strokeStyle = 'white';
  }
};

// Collect world-space surface boundary points
const collectSurfacePoints = (
  origin,
  orientation,
  describeSurface,
  C, R, Z, T,
  points = [],
  before = 0,
  after = 0,
  numerator = 0,
  divisions = 0,
  denominator = (1 << divisions),
  start = describeSurface(numerator, denominator, before, after, divisions),
  stop = describeSurface(((numerator + 1) % denominator), denominator, before, after, divisions),
) => {
  const startSurfacePoint = ray2(origin, orientation + (numerator / denominator) * TAU, start.radius);
  const levelSurfacePoint = ray2(origin, orientation + ((numerator + 1) / denominator) * TAU, start.radius);
  const projectedStartSurfacePoint = project(startSurfacePoint, C, R, Z);
  const projectedLevelSurfacePoint = project(levelSurfacePoint, C, R, Z);

  if (divisions <= 3 || distance2(projectedStartSurfacePoint, projectedLevelSurfacePoint) > T) {
    const center = describeSurface(numerator * 2 + 1, denominator * 2, start.entropy, stop.entropy, divisions);
    collectSurfacePoints(origin, orientation, describeSurface, C, R, Z, T, points, before, center.entropy, numerator * 2, divisions + 1, denominator * 2, start, center);
    collectSurfacePoints(origin, orientation, describeSurface, C, R, Z, T, points, center.entropy, after, numerator * 2 + 1, divisions + 1, denominator * 2, center, stop);
  } else {
    // Collect world-space point, not projected
    points.push(startSurfacePoint);
  }
  return points;
};

// Fill the interior with sparse white dots in screen space
const fillInterior = (plotter, worldPoints, origin, orientation, C, R, Z) => {
  if (worldPoints.length < 3) return;

  plotter.save();
  plotter.beginPath();
  const p0 = project(worldPoints[0], C, R, Z);
  plotter.moveTo(p0.x, p0.y);
  for (let i = 1; i < worldPoints.length; i++) {
    const p = project(worldPoints[i], C, R, Z);
    plotter.lineTo(p.x, p.y);
  }
  plotter.closePath();
  plotter.clip();

  // Project the origin to get screen-space center
  const projectedOrigin = project(origin, C, R, Z);

  // Draw a rotated grid of dots in screen space
  const dotSpacing = 12;
  const dotSize = 1;
  const gridExtent = 500; // Cover enough area

  plotter.fillStyle = 'white';

  // Rotation matrix components
  const cos = Math.cos(orientation);
  const sin = Math.sin(orientation);

  for (let gx = -gridExtent; gx <= gridExtent; gx += dotSpacing) {
    for (let gy = -gridExtent; gy <= gridExtent; gy += dotSpacing) {
      // Offset based on distance from edge (approximated by distance from center)
      const distFromCenter = Math.hypot(gx, gy);
      const depthOffset = distFromCenter * 0.02;

      // Apply rotation around origin with depth-based offset
      const offsetGx = gx + depthOffset;
      const rx = offsetGx * cos - gy * sin;
      const ry = offsetGx * sin + gy * cos;

      const screenX = projectedOrigin.x + rx;
      const screenY = projectedOrigin.y + ry;

      plotter.beginPath();
      plotter.arc(screenX, screenY, dotSize, 0, TAU);
      plotter.fill();
    }
  }

  plotter.restore();
};

const drawSurfaceDetail = (
  plotter,
  origin,
  orientation,
  describeSurface,
  C, R, Z, T,
  before = 0,
  after = 0,
  numerator = 0,
  divisions = 0,
  denominator = (1 << divisions),
  start = describeSurface(numerator, denominator, before, after, divisions),
  startSurfacePoint = ray2(origin, orientation + (numerator / denominator) * TAU, start.radius),
  startDepthPoint = ray2(origin, orientation + (numerator / denominator) * TAU, start.radius * (1 - 1 / denominator)),
  stop = describeSurface(((numerator + 1) % denominator), denominator, before, after, divisions),
  stopSurfacePoint = ray2(origin, orientation + ((numerator + 1) / denominator) * TAU, stop.radius),
) => {
  const levelSurfacePoint = ray2(origin, orientation + ((numerator + 1) / denominator) * TAU, start.radius);
  const projectedStartSurfacePoint = project(startSurfacePoint, C, R, Z);
  const projectedLevelSurfacePoint = project(levelSurfacePoint, C, R, Z);
  if (divisions <= 3 || distance2(projectedStartSurfacePoint, projectedLevelSurfacePoint) > T) {
    const center = describeSurface(numerator * 2 + 1, denominator * 2, start.entropy, stop.entropy, divisions);
    drawSurfaceDetail(
      plotter,
      origin,
      orientation,
      describeSurface,
      C, R, Z, T,
      before,
      center.entropy,
      numerator * 2,
      divisions + 1,
      denominator * 2,
      start,
      startSurfacePoint,
      startDepthPoint,
      center,
    );
    drawSurfaceDetail(
      plotter,
      origin,
      orientation,
      describeSurface,
      C, R, Z, T,
      center.entropy,
      after,
      numerator * 2 + 1,
      divisions + 1,
      denominator * 2,
      center,
      undefined,
      undefined,
      stop,
      stopSurfacePoint,
    );
  } else {
    // Check if this segment is in the landing zone
    const segmentAngle = (numerator / denominator) * TAU;
    let inLandingZone = false;
    if (describeSurface.landingZoneAngle !== undefined) {
      let distToLandingZone = Math.abs(segmentAngle - describeSurface.landingZoneAngle);
      if (distToLandingZone > Math.PI) distToLandingZone = TAU - distToLandingZone;
      inLandingZone = distToLandingZone < describeSurface.landingZoneWidth / 2;
    }

    plotter.strokeStyle = inLandingZone ? '#0f0' : 'white';
    drawLine(plotter, startDepthPoint, startSurfacePoint, C, R, Z, T);
    drawLine(plotter, startSurfacePoint, stopSurfacePoint, C, R, Z, T);
  }
};

const drawAbstractSurface = (plotter, center, orientation, surfaceRadius, C, R, Z, T) => {
  // Measure spread radially (independent of rotation) using projection formula directly
  const distanceToCenter = Math.hypot(center.x, center.y);
  const centerProjectedRadius = Math.atan2(distanceToCenter, Z) / TAU * 4 * R;
  const nearEdgeProjectedRadius = Math.atan2(Math.max(0, distanceToCenter - surfaceRadius), Z) / TAU * 4 * R;
  const spread = centerProjectedRadius - nearEdgeProjectedRadius;

  if (spread > T * 2) {
    return true;
  }
  const projectedCenter = project(center, C, R, Z);
  plotter.beginPath();
  plotter.moveTo(projectedCenter.x, projectedCenter.y);
  plotter.arc(projectedCenter.x, projectedCenter.y, T, orientation, orientation + TAU, false);
  plotter.stroke();
  return false;
};

const drawSurface = (plotter, origin, orientation, describeSurface, C, R, Z, T) => {
  const r = describeSurface(0, 1, 0, 0, 0).radius;
  const concrete = drawAbstractSurface(plotter, origin, orientation, r, C, R, Z, T);
  if (concrete) {
    // Collect world-space surface boundary points and fill interior
    const surfacePoints = collectSurfacePoints(origin, orientation, describeSurface, C, R, Z, T);
    fillInterior(plotter, surfacePoints, origin, orientation, C, R, Z);

    // Draw surface outline (white) on top
    drawSurfaceDetail(plotter, origin, orientation, describeSurface, C, R, Z, T);
  }
};

const makeAsteroidSurface = (state, min, max, numPeaks = 12) => {
  // Generate random peaks using consistent seeding
  const peaks = [];
  const peakState = new Uint32Array(seed);
  churn(peakState);

  // Landing zone at a consistent random angle
  const landingZoneAngle = random(peakState) * TAU;
  const landingZoneWidth = TAU / 16; // Width of the landing zone

  for (let i = 0; i < numPeaks; i++) {
    peaks.push({
      angle: random(peakState) * TAU,
      height: random(peakState) * 0.5 + 0.1,  // 10-60% of radius range
      sharpness: random(peakState) * 20 + 8,  // Controls peak width (higher = sharper)
    });
  }

  const range = max - min;

  // Return function matching expected interface
  const describeSurface = (n, d, _before, _after, _l) => {
    const angle = (n / d) * TAU;

    // Check if we're in the landing zone
    let distToLandingZone = Math.abs(angle - landingZoneAngle);
    if (distToLandingZone > Math.PI) distToLandingZone = TAU - distToLandingZone;
    const inLandingZone = distToLandingZone < landingZoneWidth / 2;

    let radius = min;
    for (const peak of peaks) {
      // Angular distance with wraparound
      let dist = Math.abs(angle - peak.angle);
      if (dist > Math.PI) dist = TAU - dist;

      // Suppress peaks in the landing zone for a flatter surface
      let peakContribution = peak.height;
      if (inLandingZone) {
        // Smoothly reduce peak contribution in landing zone
        const landingZoneFactor = distToLandingZone / (landingZoneWidth / 2);
        peakContribution *= landingZoneFactor;
      }

      // Gaussian falloff - sharp peaks, smooth valleys
      const contribution = peakContribution * Math.exp(-peak.sharpness * dist * dist);
      radius += range * contribution;
    }

    // entropy not used in this model, return normalized radius
    return { entropy: (radius - min) / range, radius };
  };

  // Attach landing zone info to the function for rendering
  describeSurface.landingZoneAngle = landingZoneAngle;
  describeSurface.landingZoneWidth = landingZoneWidth;

  return describeSurface;
};

export const radiusAt = (describeSurface, meridian, T) => {
  let n = 0;
  let d = 1;
  let l = 0;
  let before = 0;
  let after = 0;
  let description = describeSurface(n, d, before, after, l);
  before = description.entropy;
  after = description.entropy;
  while (TAU * description.radius / d > T / 2) {
    n*=2;
    d*=2;
    l+=1;
    description = describeSurface(n+1, d, before, after, l);
    if (TAU * (n+1) / d > meridian) {
      after = description.entropy;
    } else {
      before = description.entropy;
      n+=1;
    }
  }
  return description.radius;
};

const main = () => {
  const distanceFormat = new Intl.NumberFormat([], {
    maximumFractionDigits: 0,
  });
  const speedFormat = new Intl.NumberFormat([], {
    maximumFractionDigits: 3,
  });

  const $plotter = document.querySelector('#plotter');
  $plotter.width = window.innerWidth;
  $plotter.height = window.innerHeight;

  const $info = document.querySelector('#info');

  const plotter = $plotter.getContext('2d');

  const state = new Uint32Array(seed);

  const viewportSizePx = {x: window.innerWidth, y: window.innerHeight};
  const margin = 20;

  // Z is the distance from the center that corresponds to
  // 50% the radius of the projection, or the elevation of an
  // observer over the center that can see to infinity in all
  // directions within a circle of radius 2 Z.
  const Z = 100;

  // T is the threshold for partitioning a segment, used to
  // reduce level of detail for indistinct features.
  const T = 20;

  // R is the radius of the view.
  const R = Math.min(viewportSizePx.x, viewportSizePx.y) / 2 - margin;

  // C is the center of the view.
  const C = scale2(viewportSizePx, 0.5);

  const describeAsteroidSurface = makeAsteroidSurface(state, 500, 1000);
  // const describeAsteroidSurface = (n, d, before, after, l) => {
  //   return { entropy: 0, radius: 500 + n / d * 500 };
  // }
  // const spikeySurfaceRadius = makeSpikeySurface(state, 10, 15);
  // const pentasterSurfaceRadius = makeSinusoidSurface(13, 2, 5);
  // const starSurfaceRadius = makeRandomSurface(1000, 1100);

  let vesselPosition = {x: 0, y: 0, a: 0};
  let vesselVelocity = {x: 0, y: 0, a: 0};

  let targetPosition = {x: 1000, y: 0, a: TAU/2};
  let targetVelocity = {x: 1 / 1000, y: 0, a: 0};

  let t = performance.now();

  const keys = {
    __proto__: null,
    q: 0,
    w: 0,
    e: 0,
    a: 0,
    s: 0,
    d: 0,
    ' ': 0, // Spacebar for anchor
    ArrowUp: 0,
    ArrowDown: 0,
    ArrowLeft: 0,
    ArrowRight: 0,
  };

  // Anchor state
  let anchorState = {
    deployed: false,
    attachedAngle: 0, // Angle on asteroid where anchor is attached (in asteroid's local coords)
  };

  /**
   * @param {KeyboardEvent} event
   */
  const onKeyDown = event => {
    const { key, repeat, metaKey } = event;
    if (repeat || metaKey) return;
    if (key in keys) {
      keys[key] = 1;
      event.stopPropagation();

      // Toggle anchor on spacebar press
      if (key === ' ') {
        if (!anchorState.deployed) {
          // Deploy anchor - calculate attachment point on asteroid
          const direction = direction2(targetPosition, vesselPosition);
          const meridian = (direction + targetPosition.a + TAU) % TAU;
          const surfaceRadius = radiusAt(describeAsteroidSurface, meridian, T);
          const distanceToSurface = distance2(vesselPosition, targetPosition) - surfaceRadius;

          // Only attach if close enough (within 50 units)
          if (distanceToSurface < 50) {
            anchorState.deployed = true;
            anchorState.attachedAngle = meridian; // Store in asteroid's local coordinates
          }
        } else {
          // Retract anchor
          anchorState.deployed = false;
        }
      }
    }
  };

  /**
   * @param {KeyboardEvent} event
   */
  const onKeyUp = event => {
    const { key, repeat, metaKey } = event;
    if (repeat || metaKey) return;
    if (key in keys) {
      keys[key] = 0;
      event.stopPropagation();
    }
  };

  const onBlur = _event => {
    for (const key in keys) {
      keys[key] = false;
    }
  };

  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
  window.addEventListener('blur', onBlur);

  const simulate = () => {
    const t2 = performance.now();
    const dt = t2 - t;
    t = t2;

    setTimeout(simulate, 100);

    const vesselThrust = {
      x: (keys.w + keys.ArrowUp - keys.s - keys.ArrowDown) / 100000,
      y: (keys.d + keys.ArrowRight - keys.a - keys.ArrowLeft) / 100000,
    };
    const vesselImpulse = {
      ...m2d.transform(
        vesselThrust,
        m2d.rotate(-vesselPosition.a),
      ),
      a: (keys.q - keys.e) * TAU / 100000000,
    };

    if (anchorState.deployed) {
      // When anchored, thrust applies torque to the asteroid
      // Calculate the anchor point in world space
      const anchorWorldAngle = anchorState.attachedAngle - targetPosition.a;
      const anchorRadius = radiusAt(describeAsteroidSurface, anchorState.attachedAngle, T);
      const anchorPoint = ray2(targetPosition, anchorWorldAngle, anchorRadius);

      // Tether vector from asteroid center to vessel
      const tetherVector = sub2(vesselPosition, targetPosition);
      const tetherLength = Math.hypot(tetherVector.x, tetherVector.y);

      // Apply thrust as torque to asteroid (simplified physics)
      // The thrust perpendicular to the tether creates torque
      const thrustMagnitude = Math.hypot(vesselImpulse.x, vesselImpulse.y);
      const thrustAngle = Math.atan2(vesselImpulse.y, vesselImpulse.x);
      const tetherAngle = Math.atan2(tetherVector.y, tetherVector.x);
      const angleDiff = thrustAngle - tetherAngle;

      // Torque is proportional to perpendicular component of thrust times tether length
      const torque = thrustMagnitude * Math.sin(angleDiff) * tetherLength * 0.00001;
      targetVelocity = { ...targetVelocity, a: targetVelocity.a + torque * dt };

      // Vessel rotates with the asteroid (constrained by tether)
      // Update vessel position to maintain tether length from anchor point
      const currentDist = distance2(vesselPosition, anchorPoint);
      if (currentDist > 0.1) {
        const toAnchor = direction2(vesselPosition, anchorPoint);
        // Apply some constraint force to keep vessel at tether length
        vesselPosition = ray2(anchorPoint, toAnchor + Math.PI, Math.min(currentDist, anchorRadius + 30));
      }

      // Vessel still rotates independently
      vesselVelocity = { ...vesselVelocity, a: vesselVelocity.a + vesselImpulse.a * dt };
      vesselPosition = { ...vesselPosition, a: vesselPosition.a + vesselVelocity.a * dt };
    } else {
      // Normal flight - thrust affects vessel
      vesselVelocity = add2a(vesselVelocity, scale2a(vesselImpulse, dt));
      vesselPosition = add2a(vesselPosition, scale2a(vesselVelocity, dt));
    }

    targetPosition = add2a(targetPosition, scale2a(targetVelocity, dt));
  };

  const draw = () => {
    requestAnimationFrame(draw);
    plotter.fillStyle = 'solid black';
    plotter.fillRect(0, 0, $plotter.width, $plotter.height);

    const viewMatrix = m2d.compose(
      m2d.scale(-1),
      m2d.translate(vesselPosition),
      m2d.rotate(vesselPosition.a),
      m2d.rotate(TAU/4),
    );

    plotter.strokeStyle = 'white';
    const thrustState = {
      forward: keys.w || keys.ArrowUp,
      backward: keys.s || keys.ArrowDown,
      left: keys.a || keys.ArrowLeft,
      right: keys.d || keys.ArrowRight,
      rotateLeft: keys.q,
      rotateRight: keys.e,
    };
    drawVessel(plotter, m2d.transform(vesselPosition, viewMatrix), vesselPosition.a - vesselPosition.a - TAU/4, 10, C, R, Z, T, thrustState);
    // drawVessel(plotter, m2d.transform(targetPosition, viewMatrix), targetPosition.a - vesselPosition.a - TAU/4, 0.5, C, R, Z, T);
    drawSurface(plotter, m2d.transform(targetPosition, viewMatrix), vesselPosition.a - targetPosition.a - TAU/4, describeAsteroidSurface, C, R, Z, T);

    // Draw tether if anchored
    if (anchorState.deployed) {
      const anchorWorldAngle = anchorState.attachedAngle - targetPosition.a;
      const anchorRadius = radiusAt(describeAsteroidSurface, anchorState.attachedAngle, T);
      const anchorPoint = ray2(targetPosition, anchorWorldAngle, anchorRadius);
      const anchorPointView = m2d.transform(anchorPoint, viewMatrix);
      const vesselPointView = m2d.transform(vesselPosition, viewMatrix);

      // Draw tether line
      plotter.save();
      plotter.strokeStyle = '#0ff';
      const p1 = project(vesselPointView, C, R, Z);
      const p2 = project(anchorPointView, C, R, Z);
      plotter.beginPath();
      plotter.moveTo(p1.x, p1.y);
      plotter.lineTo(p2.x, p2.y);
      plotter.stroke();
      plotter.restore();

      // Draw anchor point marker
      const projectedAnchor = project(anchorPointView, C, R, Z);
      plotter.fillStyle = '#0ff';
      plotter.beginPath();
      plotter.arc(projectedAnchor.x, projectedAnchor.y, 4, 0, TAU);
      plotter.fill();
    }

    // drawSurface(plotter, {x: 2000, y: 0}, Math.random() * Math.PI * 2, starSurfaceRadius, C, R, Z, T);
    // drawSurface(plotter, {x: 0, y: 15}, planetRotation, pentasterSurfaceRadius, C, R, Z, T);
    // drawSurface(plotter, {x: 0, y: 0}, 0, spikeySurfaceRadius, C, R, Z, T);
    //
    const vesselBearingToTarget = sub2(vesselPosition, targetPosition);
    const vesselDirectionToTarget = (direction2(targetPosition, vesselPosition) + TAU) % TAU;
    const vesselDirectionAcrossTarget = perp2(vesselBearingToTarget);
    const vesselSpeedOnBearingToTarget = dot2(vesselVelocity, vesselBearingToTarget);
    const vesselSpeedAcrossBearingToTarget = dot2(vesselVelocity, vesselDirectionAcrossTarget);
    const direction = (direction2(targetPosition, vesselPosition) + TAU) % TAU;
    const meridian = (direction + targetPosition.a + TAU) % TAU;
    const elevation = radiusAt(describeAsteroidSurface, meridian, T);
    const range = distance2(vesselPosition, targetPosition) - elevation - 10;
    const rangeCircumference = TAU * range;
    const surfaceCircumference = TAU * elevation;
    const surfaceRangeSpeed = (vesselSpeedAcrossBearingToTarget / rangeCircumference - targetVelocity.a);
    const surfaceSpeed = surfaceRangeSpeed / rangeCircumference * surfaceCircumference;

    // Draw visual HUD
    const hudData = {
      range: range,
      rangeSpeed: vesselSpeedOnBearingToTarget,
      heading: ((vesselDirectionToTarget - vesselPosition.a + TAU/2) % TAU / TAU),
      spin: -vesselVelocity.a % TAU / TAU,
      anchored: anchorState.deployed,
    };
    drawHUD(plotter, viewportSizePx, hudData);

    $info.innerText = `\
    range to target: ${distanceFormat.format(range)}m
    range speed ${vesselSpeedOnBearingToTarget.toFixed(1)}m/s
    range ortho: ${vesselSpeedAcrossBearingToTarget.toFixed(1)}m/s
    heading: ${((vesselDirectionToTarget - vesselPosition.a + TAU/2) % TAU / TAU).toFixed(2)}τ
    spin: ${(-vesselVelocity.a%TAU/TAU*1000).toFixed(4)}τ/s
    surface position: ${((vesselDirectionToTarget + targetPosition.a + TAU) % TAU / TAU).toFixed(2)}τ
    surface speed: ${surfaceSpeed.toFixed(1)}m/s
    anchored: ${anchorState.deployed ? 'YES' : 'NO'}
    `;
  };

  draw();
  setTimeout(simulate, 100);
};

main();

