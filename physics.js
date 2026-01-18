import { distance2, direction2, ray2, dot2 } from './geometry2d.js';
import { add2a, scale2a } from './geometry2a.js';

export const TAU = Math.PI * 2;

/**
 * @typedef {{x: number, y: number, a: number}} Pose
 * @typedef {{deployed: boolean, attachedAngle: number, tetherLength: number, relativeHeading: number}} AnchorState
 * @typedef {{radiusAt: (angle: number) => number}} Surface
 */

// Physics constants
const VESSEL_MASS = 1;
const ASTEROID_DENSITY = 1e-9; // Mass = density * r³, tuned for feel

/**
 * Calculate the world-space position of an anchor point on a surface.
 * @param {Pose} targetPosition - The target (asteroid) position and orientation
 * @param {number} attachedAngle - The angle in asteroid-local coordinates where anchor is attached
 * @param {Surface} surface - Surface description with radiusAt method
 * @returns {{point: {x: number, y: number}, radius: number, worldAngle: number}}
 */
export const getAnchorPoint = (targetPosition, attachedAngle, surface) => {
  const worldAngle = attachedAngle + targetPosition.a;
  const radius = surface.radiusAt(attachedAngle);
  const point = ray2(targetPosition, worldAngle, radius);
  return { point, radius, worldAngle };
};

/**
 * Check if anchor can be deployed based on distance and relative velocity.
 * @param {Pose} vesselPosition
 * @param {{x: number, y: number, a: number}} vesselVelocity
 * @param {Pose} targetPosition
 * @param {{x: number, y: number, a: number}} targetVelocity
 * @param {Surface} surface
 * @param {{maxDistance: number, maxRelativeSpeed: number}} constraints
 * @returns {{canDeploy: boolean, localAngle: number, distanceToSurface: number, relativeSpeed: number}}
 */
export const checkAnchorDeployment = (
  vesselPosition,
  vesselVelocity,
  targetPosition,
  targetVelocity,
  surface,
  constraints = { maxDistance: 20, maxRelativeSpeed: 0.5 }
) => {
  const direction = direction2(targetPosition, vesselPosition);
  const localAngle = ((direction - targetPosition.a) % TAU + TAU) % TAU;
  const surfaceRadius = surface.radiusAt(localAngle);
  const distanceToSurface = distance2(vesselPosition, targetPosition) - surfaceRadius;

  // Calculate relative velocity to surface point
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

  const canDeploy = distanceToSurface < constraints.maxDistance &&
                    relativeSpeed < constraints.maxRelativeSpeed;

  return { canDeploy, localAngle, distanceToSurface, relativeSpeed };
};

/**
 * Update physics state for one simulation step when anchored.
 * Vessel state is defined RELATIVE to asteroid - world coords are derived.
 * Thrust affects asteroid based on mass ratio.
 *
 * @param {Pose} vesselPosition
 * @param {{x: number, y: number, a: number}} vesselVelocity
 * @param {Pose} targetPosition
 * @param {{x: number, y: number, a: number}} targetVelocity
 * @param {AnchorState} anchorState
 * @param {Surface} surface
 * @param {{x: number, y: number, a: number}} vesselImpulse
 * @param {number} dt - Time delta in milliseconds
 * @param {number} meanRadius - Mean radius of asteroid for mass calculation
 * @returns {{vesselPosition: Pose, vesselVelocity: {x: number, y: number, a: number}, targetPosition: Pose, targetVelocity: {x: number, y: number, a: number}, anchorState: AnchorState}}
 */
export const updateAnchoredPhysics = (
  vesselPosition,
  vesselVelocity,
  targetPosition,
  targetVelocity,
  anchorState,
  surface,
  vesselImpulse,
  dt,
  meanRadius
) => {
  // Compute masses - asteroid mass proportional to cube of mean radius
  const asteroidMass = ASTEROID_DENSITY * meanRadius * meanRadius * meanRadius;
  const momentOfInertia = 0.4 * asteroidMass * meanRadius * meanRadius; // Sphere approximation

  // Apply thrust to asteroid (both linear and angular)
  const thrustMagnitude = Math.hypot(vesselImpulse.x, vesselImpulse.y);
  let newTargetVelocity = { ...targetVelocity };

  // Get anchor parameters for torque calculation
  const anchorRadius = surface.radiusAt(anchorState.attachedAngle);
  const vesselDistance = anchorRadius + anchorState.tetherLength;
  const anchorWorldAngle = anchorState.attachedAngle + targetPosition.a;

  if (thrustMagnitude > 0) {
    // Linear acceleration: a = F/m
    // Thrust from vessel pushes the asteroid
    const linearAccelX = vesselImpulse.x * VESSEL_MASS / asteroidMass;
    const linearAccelY = vesselImpulse.y * VESSEL_MASS / asteroidMass;

    // Angular acceleration from torque
    // Torque = r × F = |r| * |F| * sin(θ)
    const thrustAngle = Math.atan2(vesselImpulse.y, vesselImpulse.x);
    const angleDiff = thrustAngle - anchorWorldAngle;
    const torque = thrustMagnitude * Math.sin(angleDiff) * vesselDistance * VESSEL_MASS;
    const angularAccel = torque / momentOfInertia;

    newTargetVelocity = {
      x: targetVelocity.x + linearAccelX * dt,
      y: targetVelocity.y + linearAccelY * dt,
      a: targetVelocity.a + angularAccel * dt,
    };
  }

  // Update asteroid position
  const newTargetPosition = add2a(targetPosition, scale2a(newTargetVelocity, dt));

  // Update relative heading from angular impulse (Q/E controls)
  const newRelativeHeading = anchorState.relativeHeading + vesselImpulse.a * dt;

  // Derive vessel world position from asteroid state + relative coords
  const newAnchorWorldAngle = anchorState.attachedAngle + newTargetPosition.a;
  const newVesselPosition = {
    ...ray2(newTargetPosition, newAnchorWorldAngle, vesselDistance),
    a: newTargetPosition.a + newRelativeHeading,  // World heading = asteroid rotation + relative
  };

  // Vessel velocity while tethered: linear velocity matches asteroid + rotational component
  // (This is used when untethering to give correct initial velocity)
  const rotationalSpeed = newTargetVelocity.a * vesselDistance;
  const tangentAngle = newAnchorWorldAngle + TAU / 4;
  const newVesselVelocity = {
    x: newTargetVelocity.x + Math.cos(tangentAngle) * rotationalSpeed,
    y: newTargetVelocity.y + Math.sin(tangentAngle) * rotationalSpeed,
    a: 0,  // Relative angular velocity is in anchorState, not here
  };

  // Return updated anchor state with new relative heading
  const newAnchorState = {
    ...anchorState,
    relativeHeading: newRelativeHeading,
  };

  return {
    vesselPosition: newVesselPosition,
    vesselVelocity: newVesselVelocity,
    targetPosition: newTargetPosition,
    targetVelocity: newTargetVelocity,
    anchorState: newAnchorState,
  };
};

/**
 * Update physics state for one simulation step during normal flight.
 * @param {Pose} vesselPosition
 * @param {{x: number, y: number, a: number}} vesselVelocity
 * @param {{x: number, y: number, a: number}} vesselImpulse
 * @param {number} dt - Time delta in milliseconds
 * @returns {{vesselPosition: Pose, vesselVelocity: {x: number, y: number, a: number}}}
 */
export const updateFlightPhysics = (vesselPosition, vesselVelocity, vesselImpulse, dt) => {
  const newVesselVelocity = add2a(vesselVelocity, scale2a(vesselImpulse, dt));
  const newVesselPosition = add2a(vesselPosition, scale2a(newVesselVelocity, dt));
  return {
    vesselPosition: newVesselPosition,
    vesselVelocity: newVesselVelocity,
  };
};

/**
 * Update target position based on its velocity.
 * @param {Pose} targetPosition
 * @param {{x: number, y: number, a: number}} targetVelocity
 * @param {number} dt
 * @returns {Pose}
 */
export const updateTargetPosition = (targetPosition, targetVelocity, dt) => {
  return add2a(targetPosition, scale2a(targetVelocity, dt));
};

/**
 * Check if vessel has collided with asteroid surface.
 * @param {Pose} vesselPosition
 * @param {{x: number, y: number, a: number}} vesselVelocity
 * @param {Pose} targetPosition
 * @param {{x: number, y: number, a: number}} targetVelocity
 * @param {Surface} surface
 * @param {number} vesselRadius - Radius of vessel for collision detection
 * @returns {{collided: boolean, distanceToSurface: number, localAngle: number, relativeSpeed: number}}
 */
export const checkCollision = (
  vesselPosition,
  vesselVelocity,
  targetPosition,
  targetVelocity,
  surface,
  vesselRadius = 10
) => {
  const direction = direction2(targetPosition, vesselPosition);
  const localAngle = ((direction - targetPosition.a) % TAU + TAU) % TAU;
  const surfaceRadius = surface.radiusAt(localAngle);
  const distanceToSurface = distance2(vesselPosition, targetPosition) - surfaceRadius;

  // Calculate relative velocity to surface point
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

  // Collision if vessel is inside or touching surface
  const collided = distanceToSurface <= vesselRadius;

  return { collided, distanceToSurface, localAngle, relativeSpeed };
};

/**
 * Apply bounce physics to vessel after collision.
 * Reflects velocity relative to surface normal with energy loss.
 * @param {Pose} vesselPosition
 * @param {{x: number, y: number, a: number}} vesselVelocity
 * @param {Pose} targetPosition
 * @param {{x: number, y: number, a: number}} targetVelocity
 * @param {Surface} surface
 * @param {number} localAngle - Angle in asteroid-local coordinates where collision occurred
 * @param {number} restitution - Bounce coefficient (0-1, where 1 is perfect bounce)
 * @returns {{vesselPosition: Pose, vesselVelocity: {x: number, y: number, a: number}}}
 */
export const applyBounce = (
  vesselPosition,
  vesselVelocity,
  targetPosition,
  targetVelocity,
  surface,
  localAngle,
  restitution = 0.6
) => {
  // Get surface normal at collision point
  const worldAngle = localAngle + targetPosition.a;
  const surfaceRadius = surface.radiusAt(localAngle);
  const surfacePoint = ray2(targetPosition, worldAngle, surfaceRadius);

  // Normal points from surface center to collision point
  const normal = {
    x: (vesselPosition.x - targetPosition.x) / distance2(vesselPosition, targetPosition),
    y: (vesselPosition.y - targetPosition.y) / distance2(vesselPosition, targetPosition),
  };

  // Calculate surface velocity at collision point
  const tangentSpeed = targetVelocity.a * surfaceRadius;
  const tangentDir = worldAngle + TAU / 4;
  const surfaceVelocity = {
    x: targetVelocity.x + Math.cos(tangentDir) * tangentSpeed,
    y: targetVelocity.y + Math.sin(tangentDir) * tangentSpeed,
  };

  // Relative velocity
  const relativeVelocity = {
    x: vesselVelocity.x - surfaceVelocity.x,
    y: vesselVelocity.y - surfaceVelocity.y,
  };

  // Project relative velocity onto normal
  const normalSpeed = dot2(relativeVelocity, normal);
  
  // Only bounce if moving toward surface
  if (normalSpeed < 0) {
    // Reflect velocity: v' = v - 2 * (v · n) * n
    const reflectedVelocity = {
      x: relativeVelocity.x - 2 * normalSpeed * normal.x * restitution,
      y: relativeVelocity.y - 2 * normalSpeed * normal.y * restitution,
    };

    // Add back surface velocity
    const newVesselVelocity = {
      x: surfaceVelocity.x + reflectedVelocity.x,
      y: surfaceVelocity.y + reflectedVelocity.y,
      a: vesselVelocity.a,
    };

    // Push vessel outside surface
    const pushDistance = surfaceRadius + 10.1; // vesselRadius + small margin
    const newVesselPosition = {
      ...ray2(targetPosition, worldAngle, pushDistance),
      a: vesselPosition.a,
    };

    return {
      vesselPosition: newVesselPosition,
      vesselVelocity: newVesselVelocity,
    };
  }

  // No bounce needed, return original
  return {
    vesselPosition,
    vesselVelocity,
  };
};
