
import * as m2d from './matrix2d.js';
import { sub2, dot2, perp2, lerp2, distance2, direction2, ray2, scale2 } from './geometry2d.js';
import { churn, fold, random } from './xorshift128.js';
import { radiusAt } from './surface.js';
import {
  updateAnchoredPhysics,
  updateFlightPhysics,
  updateTargetPosition,
  checkCollision,
  applyBounce,
} from './physics.js';

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

  // Project the origin to get screen-space asteroid center for rotation reference
  const projectedOrigin = project(origin, C, R, Z);

  // Draw a rotated grid of dots covering the entire screen
  const dotSpacing = 12;
  const dotSize = 1;
  const gridExtent = R * 4; // Cover entire viewport even when offset from center

  plotter.fillStyle = 'white';

  // Rotation matrix components (rotate around asteroid center)
  const cos = Math.cos(orientation);
  const sin = Math.sin(orientation);

  for (let gx = -gridExtent; gx <= gridExtent; gx += dotSpacing) {
    for (let gy = -gridExtent; gy <= gridExtent; gy += dotSpacing) {
      // Distance from asteroid center in grid space for depth offset
      const distFromOrigin = Math.hypot(gx - (projectedOrigin.x - C.x), gy - (projectedOrigin.y - C.y));
      const depthOffset = distFromOrigin * 0.02;

      // Apply rotation around asteroid center
      const relX = gx - (projectedOrigin.x - C.x);
      const relY = gy - (projectedOrigin.y - C.y);
      const rotX = relX * cos - relY * sin + depthOffset * cos;
      const rotY = relX * sin + relY * cos + depthOffset * sin;

      const screenX = C.x + (projectedOrigin.x - C.x) + rotX;
      const screenY = C.y + (projectedOrigin.y - C.y) + rotY;

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

const main = () => {
  const distanceFormat = new Intl.NumberFormat([], {
    maximumFractionDigits: 0,
  });
  const speedFormat = new Intl.NumberFormat([], {
    maximumFractionDigits: 3,
  });
  const zFormat = new Intl.NumberFormat([], {
    maximumFractionDigits: 0,
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
  // Z is controlled by a slider, logarithmically from 100 to 100000
  let Z = 100;

  // Z slider setup
  const $zSlider = document.querySelector('#z-slider');
  let isDraggingZ = false;
  const updateZ = () => {
    // Slider value is 0-100, convert to logarithmic scale: 100 to solar system diameter
    // Solar system diameter ≈ 1 × 10^16 meters (including Oort cloud)
    // Z = 100 * 10^(slider_value * log10(10^16 / 100))
    // log10(10^16 / 100) = log10(10^14) = 14
    const sliderValue = parseFloat($zSlider.value) / 100; // 0 to 1
    Z = 100 * Math.pow(10, sliderValue * 14); // 100 * 10^(0 to 14) = 100 to 10^16
  };
  $zSlider.addEventListener('input', updateZ);
  $zSlider.addEventListener('mousedown', () => { isDraggingZ = true; });
  $zSlider.addEventListener('touchstart', () => { isDraggingZ = true; });
  $zSlider.addEventListener('mouseup', () => { isDraggingZ = false; });
  $zSlider.addEventListener('touchend', () => { isDraggingZ = false; });
  $zSlider.addEventListener('mouseleave', () => { isDraggingZ = false; });
  updateZ(); // Initialize Z from slider

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

  // Surface adapter for physics library
  const surface = {
    radiusAt: (angle) => radiusAt(describeAsteroidSurface, angle, T),
  };

  // Mean radius for mass calculations (average of min/max)
  const meanRadius = 750;

  // Game state
  const GAME_STATE = {
    PLAYING: 'playing',
    PAUSED: 'paused',  // High-energy collision, physics paused
    GAME_OVER: 'game_over',
  };
  let gameState = GAME_STATE.PLAYING;
  let collisionInfo = null;

  // Initial positions
  const initialVesselPosition = {x: 0, y: 0, a: 0};
  const initialVesselVelocity = {x: 0, y: 0, a: 0};
  const initialTargetPosition = {x: 1000, y: 0, a: TAU/2};
  const initialTargetVelocity = {x: 1 / 1000, y: 0, a: 0};

  let vesselPosition = {...initialVesselPosition};
  let vesselVelocity = {...initialVesselVelocity};

  let targetPosition = {...initialTargetPosition};
  let targetVelocity = {...initialTargetVelocity};

  let t = performance.now();

  // Collision thresholds
  const BOUNCE_THRESHOLD = 5.0; // m/s - below this, bounce; above, explode

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
    attachedAngle: 0,    // Angle on asteroid where anchor is attached (in asteroid's local coords)
    tetherLength: 0,     // Distance from anchor point to vessel at deployment
    relativeHeading: 0,  // Vessel heading relative to asteroid orientation
  };

  // Restart game function
  const restartGame = () => {
    vesselPosition = {...initialVesselPosition};
    vesselVelocity = {...initialVesselVelocity};
    targetPosition = {...initialTargetPosition};
    targetVelocity = {...initialTargetVelocity};
    anchorState = {
      deployed: false,
      attachedAngle: 0,
      tetherLength: 0,
      relativeHeading: 0,
    };
    gameState = GAME_STATE.PLAYING;
    collisionInfo = null;
  };

  /**
   * @param {KeyboardEvent} event
   */
  const onKeyDown = event => {
    const { key, repeat, metaKey } = event;
    if (repeat || metaKey) return;

    // Handle restart when paused/game over
    if ((gameState === GAME_STATE.PAUSED || gameState === GAME_STATE.GAME_OVER) && key === ' ') {
      restartGame();
      event.stopPropagation();
      return;
    }

    if (key in keys) {
      keys[key] = 1;
      event.stopPropagation();

      // Toggle anchor on spacebar press (only when playing)
      if (key === ' ' && gameState === GAME_STATE.PLAYING) {
        if (!anchorState.deployed) {
          // Deploy anchor - calculate attachment point on asteroid
          const direction = direction2(targetPosition, vesselPosition);
          // Convert world angle to asteroid-local angle
          const localAngle = (direction - targetPosition.a + TAU) % TAU;
          const surfaceRadius = radiusAt(describeAsteroidSurface, localAngle, T);
          const distanceToSurface = distance2(vesselPosition, targetPosition) - surfaceRadius;

          // Calculate relative velocity to surface point
          // Surface point velocity = asteroid center velocity + rotational velocity
          const tangentSpeed = targetVelocity.a * surfaceRadius;
          const tangentDir = direction + TAU / 4;
          const surfaceVelocity = {
            x: targetVelocity.x + Math.cos(tangentDir) * tangentSpeed,
            y: targetVelocity.y + Math.sin(tangentDir) * tangentSpeed,
          };
          const relativeVelocity = {
            x: vesselVelocity.x - surfaceVelocity.x,
            y: vesselVelocity.y - surfaceVelocity.y,
          };
          const relativeSpeed = Math.hypot(relativeVelocity.x, relativeVelocity.y);

          // Only attach if close enough (within 20 units) and low relative velocity
          if (distanceToSurface < 20 && relativeSpeed < 0.5) {
            anchorState.deployed = true;
            anchorState.attachedAngle = localAngle; // Store in asteroid's local coordinates
            anchorState.tetherLength = Math.max(0, distanceToSurface); // Distance from surface at deployment
            anchorState.relativeHeading = vesselPosition.a - targetPosition.a; // Vessel heading relative to asteroid
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

    // Don't simulate physics if paused or game over
    if (gameState !== GAME_STATE.PLAYING) {
      return;
    }

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
      const result = updateAnchoredPhysics(
        vesselPosition,
        vesselVelocity,
        targetPosition,
        targetVelocity,
        anchorState,
        surface,
        vesselImpulse,
        dt,
        meanRadius
      );
      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;
      anchorState = result.anchorState;
    } else {
      const result = updateFlightPhysics(vesselPosition, vesselVelocity, vesselImpulse, dt);
      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = updateTargetPosition(targetPosition, targetVelocity, dt);
    }

    // Check for collisions (only when not anchored)
    if (!anchorState.deployed) {
      const collision = checkCollision(
        vesselPosition,
        vesselVelocity,
        targetPosition,
        targetVelocity,
        surface,
        10 // vessel radius
      );

      if (collision.collided) {
        if (collision.relativeSpeed < BOUNCE_THRESHOLD) {
          // Low-energy collision: bounce
          const bounceResult = applyBounce(
            vesselPosition,
            vesselVelocity,
            targetPosition,
            targetVelocity,
            surface,
            collision.localAngle,
            0.6 // restitution coefficient
          );
          vesselPosition = bounceResult.vesselPosition;
          vesselVelocity = bounceResult.vesselVelocity;
        } else {
          // High-energy collision: pause physics
          gameState = GAME_STATE.PAUSED;
          collisionInfo = {
            relativeSpeed: collision.relativeSpeed,
            distanceToSurface: collision.distanceToSurface,
          };
        }
      }
    }
  };

  const draw = () => {
    requestAnimationFrame(draw);
    plotter.clearRect(0, 0, $plotter.width, $plotter.height);

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

    // Wrap surface drawing to isolate any clip state
    plotter.save();
    drawSurface(plotter, m2d.transform(targetPosition, viewMatrix), vesselPosition.a - targetPosition.a - TAU/4, describeAsteroidSurface, C, R, Z, T);
    plotter.restore();

    // Draw tether if anchored
    if (anchorState.deployed) {
      // Convert local angle to world angle
      const anchorWorldAngle = anchorState.attachedAngle + targetPosition.a;
      const anchorRadius = radiusAt(describeAsteroidSurface, anchorState.attachedAngle, T);
      const anchorPoint = ray2(targetPosition, anchorWorldAngle, anchorRadius);
      const anchorPointView = m2d.transform(anchorPoint, viewMatrix);

      // Vessel is at origin in view space
      const p1 = project({x: 0, y: 0}, C, R, Z);
      const p2 = project(anchorPointView, C, R, Z);

      // Only draw if coordinates are valid
      if (isFinite(p1.x) && isFinite(p1.y) && isFinite(p2.x) && isFinite(p2.y)) {
        // Draw tether line
        plotter.save();
        plotter.strokeStyle = '#0ff';
        plotter.lineWidth = 2;
        plotter.beginPath();
        plotter.moveTo(p1.x, p1.y);
        plotter.lineTo(p2.x, p2.y);
        plotter.stroke();
        plotter.restore();

        // Draw anchor point marker
        plotter.save();
        plotter.fillStyle = '#0ff';
        plotter.beginPath();
        plotter.arc(p2.x, p2.y, 3, 0, TAU);
        plotter.fill();
        plotter.restore();
      }
    }

    // drawSurface(plotter, {x: 2000, y: 0}, Math.random() * Math.PI * 2, starSurfaceRadius, C, R, Z, T);
    // drawSurface(plotter, {x: 0, y: 15}, planetRotation, pentasterSurfaceRadius, C, R, Z, T);
    // drawSurface(plotter, {x: 0, y: 0}, 0, spikeySurfaceRadius, C, R, Z, T);
    //
    const vesselBearingToTarget = sub2(vesselPosition, targetPosition);
    const vesselDirectionToTarget = (direction2(targetPosition, vesselPosition) + TAU) % TAU;
    const bearingLength = Math.hypot(vesselBearingToTarget.x, vesselBearingToTarget.y) || 1;
    const normalizedBearing = { x: vesselBearingToTarget.x / bearingLength, y: vesselBearingToTarget.y / bearingLength };
    const vesselDirectionAcrossTarget = perp2(normalizedBearing);
    const vesselSpeedOnBearingToTarget = dot2(vesselVelocity, normalizedBearing);
    const vesselSpeedAcrossBearingToTarget = dot2(vesselVelocity, vesselDirectionAcrossTarget);
    const direction = (direction2(targetPosition, vesselPosition) + TAU) % TAU;
    const meridian = (direction + targetPosition.a + TAU) % TAU;
    const elevation = radiusAt(describeAsteroidSurface, meridian, T);
    const range = Math.max(1, distance2(vesselPosition, targetPosition) - elevation - 10);
    const rangeCircumference = TAU * range;
    const surfaceCircumference = TAU * elevation;
    const surfaceRangeSpeed = (vesselSpeedAcrossBearingToTarget / rangeCircumference - targetVelocity.a);
    const surfaceSpeed = surfaceRangeSpeed / rangeCircumference * surfaceCircumference;

    // Draw Z indicator when slider is being dragged
    if (isDraggingZ) {
      plotter.save();
      const indicatorRadius = R / 2; // Half the viewport width - represents distance Z
      const infinityRadius = R; // Represents infinity in the projection
      const centerX = C.x;
      const centerY = C.y;

      // Draw first circle (distance Z)
      plotter.strokeStyle = 'rgba(0, 255, 255, 0.5)';
      plotter.lineWidth = 2;
      plotter.setLineDash([5, 5]);
      plotter.beginPath();
      plotter.arc(centerX, centerY, indicatorRadius, 0, TAU);
      plotter.stroke();
      plotter.setLineDash([]);

      // Draw radius line with arrowheads for first circle
      plotter.strokeStyle = 'rgba(0, 255, 255, 0.7)';
      plotter.lineWidth = 2;
      plotter.beginPath();
      plotter.moveTo(centerX, centerY);
      plotter.lineTo(centerX + indicatorRadius, centerY);
      plotter.stroke();

      // Arrowhead at outer end
      const arrowLength = 8;
      const arrowWidth = 6;
      plotter.beginPath();
      plotter.moveTo(centerX + indicatorRadius, centerY);
      plotter.lineTo(centerX + indicatorRadius - arrowLength, centerY - arrowWidth / 2);
      plotter.lineTo(centerX + indicatorRadius - arrowLength, centerY + arrowWidth / 2);
      plotter.closePath();
      plotter.fillStyle = 'rgba(0, 255, 255, 0.7)';
      plotter.fill();

      // Arrowhead at center
      plotter.beginPath();
      plotter.moveTo(centerX, centerY);
      plotter.lineTo(centerX + arrowLength, centerY - arrowWidth / 2);
      plotter.lineTo(centerX + arrowLength, centerY + arrowWidth / 2);
      plotter.closePath();
      plotter.fill();

      // Label for first circle with appropriate units
      plotter.fillStyle = 'rgba(0, 255, 255, 0.9)';
      plotter.font = '12px monospace';
      plotter.textAlign = 'center';
      plotter.textBaseline = 'middle';
      
      // Convert to appropriate unit
      let zValue, zUnit;
      if (Z < 1000) {
        zValue = Z;
        zUnit = 'm';
      } else if (Z < 1e6) {
        zValue = Z / 1000;
        zUnit = 'km';
      } else if (Z < 1e9) {
        zValue = Z / 1e6;
        zUnit = 'Mm';
      } else if (Z < 1e12) {
        zValue = Z / 1e9;
        zUnit = 'Gm';
      } else if (Z < 1e15) {
        zValue = Z / 1e12;
        zUnit = 'Tm';
      } else {
        zValue = Z / 1e15;
        zUnit = 'Pm';
      }
      
      const zValueFormat = new Intl.NumberFormat([], {
        maximumFractionDigits: 2,
        minimumFractionDigits: 0,
      });
      
      plotter.fillText(
        `Z: ${zValueFormat.format(zValue)} ${zUnit}`,
        centerX + indicatorRadius / 2,
        centerY - 20
      );

      // Draw second circle (infinity)
      plotter.strokeStyle = 'rgba(0, 255, 255, 0.4)';
      plotter.lineWidth = 2;
      plotter.setLineDash([3, 3]);
      plotter.beginPath();
      plotter.arc(centerX, centerY, infinityRadius, 0, TAU);
      plotter.stroke();
      plotter.setLineDash([]);

      // Label for infinity circle
      plotter.fillStyle = 'rgba(0, 255, 255, 0.9)';
      plotter.font = '12px monospace';
      plotter.textAlign = 'center';
      plotter.textBaseline = 'middle';
      plotter.fillText(
        'infinity',
        centerX + infinityRadius,
        centerY - 20
      );

      plotter.restore();
    }

    // Draw collision overlay if paused
    if (gameState === GAME_STATE.PAUSED && collisionInfo) {
      plotter.save();
      // Semi-transparent red overlay
      plotter.fillStyle = 'rgba(200, 0, 0, 0.7)';
      plotter.fillRect(0, 0, $plotter.width, $plotter.height);

      // Collision message
      plotter.fillStyle = '#fff';
      plotter.font = 'bold 24px monospace';
      plotter.textAlign = 'center';
      plotter.textBaseline = 'middle';
      const centerX = $plotter.width / 2;
      const centerY = $plotter.height / 2;
      
      plotter.fillText('COLLISION!', centerX, centerY - 40);
      plotter.font = '16px monospace';
      plotter.fillText(
        `Impact Speed: ${collisionInfo.relativeSpeed.toFixed(2)} m/s`,
        centerX,
        centerY
      );
      plotter.fillText(
        'Press SPACE to restart',
        centerX,
        centerY + 40
      );
      plotter.restore();
    }

    $info.innerText = `\
    range to target: ${distanceFormat.format(range)}m
    range speed ${vesselSpeedOnBearingToTarget.toFixed(1)}m/s
    range ortho: ${vesselSpeedAcrossBearingToTarget.toFixed(1)}m/s
    heading: ${((vesselDirectionToTarget - vesselPosition.a + TAU/2) % TAU / TAU).toFixed(2)}τ
    spin: ${(-vesselVelocity.a%TAU/TAU*1000).toFixed(4)}τ/s
    surface position: ${((vesselDirectionToTarget + targetPosition.a + TAU) % TAU / TAU).toFixed(2)}τ
    surface speed: ${surfaceSpeed.toFixed(1)}m/s
    anchored: ${anchorState.deployed ? 'YES' : 'NO'}
    Z: ${Z.toFixed(0)}
    ${gameState === GAME_STATE.PAUSED ? '\n    COLLISION - Press SPACE to restart' : ''}
    `;
  };

  draw();
  setTimeout(simulate, 100);
};

main();

