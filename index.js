
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

const drawVessel = (plotter, center, direction, radius, C, R, Z, T) => {
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

// Fill the interior as a single dark polygon, projecting world-space points
const fillInterior = (plotter, worldPoints, C, R, Z, color = '#111') => {
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

  // Fill a large rect, clipped to the polygon shape
  plotter.fillStyle = color;
  plotter.fillRect(-10000, -10000, 20000, 20000);
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
    plotter.strokeStyle = 'white';
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
    fillInterior(plotter, surfacePoints, C, R, Z);

    // Draw surface outline (white) on top
    drawSurfaceDetail(plotter, origin, orientation, describeSurface, C, R, Z, T);
  }
};

const makeAsteroidSurface = (state, min, max, numPeaks = 12) => {
  // Generate random peaks using consistent seeding
  const peaks = [];
  const peakState = new Uint32Array(seed);
  churn(peakState);

  for (let i = 0; i < numPeaks; i++) {
    peaks.push({
      angle: random(peakState) * TAU,
      height: random(peakState) * 0.5 + 0.1,  // 10-60% of radius range
      sharpness: random(peakState) * 20 + 8,  // Controls peak width (higher = sharper)
    });
  }

  const range = max - min;

  // Return function matching expected interface
  return (n, d, _before, _after, _l) => {
    const angle = (n / d) * TAU;

    let radius = min;
    for (const peak of peaks) {
      // Angular distance with wraparound
      let dist = Math.abs(angle - peak.angle);
      if (dist > Math.PI) dist = TAU - dist;

      // Gaussian falloff - sharp peaks, smooth valleys
      const contribution = peak.height * Math.exp(-peak.sharpness * dist * dist);
      radius += range * contribution;
    }

    // entropy not used in this model, return normalized radius
    return { entropy: (radius - min) / range, radius };
  };
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
      x: (keys.w - keys.s) / 100000,
      y: (keys.d - keys.a) / 100000,
    };
    const vesselImpulse = {
      ...m2d.transform(
        vesselThrust,
        m2d.rotate(-vesselPosition.a),
      ),
      a: (keys.q - keys.e) * TAU / 100000000,
    };

    vesselVelocity = add2a(vesselVelocity, scale2a(vesselImpulse, dt));
    vesselPosition = add2a(vesselPosition, scale2a(vesselVelocity, dt));
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
    drawVessel(plotter, m2d.transform(vesselPosition, viewMatrix), vesselPosition.a - vesselPosition.a - TAU/4, 10, C, R, Z, T);
    // drawVessel(plotter, m2d.transform(targetPosition, viewMatrix), targetPosition.a - vesselPosition.a - TAU/4, 0.5, C, R, Z, T);
    drawSurface(plotter, m2d.transform(targetPosition, viewMatrix), vesselPosition.a - targetPosition.a - TAU/4, describeAsteroidSurface, C, R, Z, T);

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
    $info.innerText = `\
    range to target: ${distanceFormat.format(range)}m
    range speed ${vesselSpeedOnBearingToTarget.toFixed(1)}m/s
    range ortho: ${vesselSpeedAcrossBearingToTarget.toFixed(1)}m/s
    heading: ${((vesselDirectionToTarget - vesselPosition.a + TAU/2) % TAU / TAU).toFixed(2)}τ
    spin: ${(-vesselVelocity.a%TAU/TAU*1000).toFixed(4)}τ/s
    surface position: ${((vesselDirectionToTarget + targetPosition.a + TAU) % TAU / TAU).toFixed(2)}τ
    surface speed: ${surfaceSpeed.toFixed(1)}m/s
    `;
  };

  draw();
  setTimeout(simulate, 100);
};

main();

