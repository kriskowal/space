import { test, describe } from 'node:test';
import assert from 'node:assert';
import {
  getAnchorPoint,
  checkAnchorDeployment,
  updateAnchoredPhysics,
  updateFlightPhysics,
  updateTargetPosition,
  TAU,
} from './physics.js';

// Helper to create a simple circular surface
const createCircularSurface = (radius) => ({
  radiusAt: () => radius,
});

// Default mean radius for tests
const TEST_MEAN_RADIUS = 750;

// Helper to check if a value is finite and not NaN
const isValidNumber = (n) => typeof n === 'number' && isFinite(n) && !isNaN(n);

// Helper to check if a pose has valid numbers
const isValidPose = (pose) =>
  isValidNumber(pose.x) && isValidNumber(pose.y) && isValidNumber(pose.a);

// Helper to check if a velocity has valid numbers
const isValidVelocity = (vel) =>
  isValidNumber(vel.x) && isValidNumber(vel.y) && isValidNumber(vel.a);

describe('getAnchorPoint', () => {
  test('returns valid point for simple case', () => {
    const targetPosition = { x: 100, y: 0, a: 0 };
    const attachedAngle = 0;
    const surface = createCircularSurface(50);

    const result = getAnchorPoint(targetPosition, attachedAngle, surface);

    assert.ok(isValidNumber(result.point.x), 'point.x should be valid');
    assert.ok(isValidNumber(result.point.y), 'point.y should be valid');
    assert.ok(isValidNumber(result.radius), 'radius should be valid');
    assert.ok(isValidNumber(result.worldAngle), 'worldAngle should be valid');
    assert.strictEqual(result.radius, 50);
  });

  test('world angle rotates with target orientation', () => {
    const surface = createCircularSurface(50);
    const attachedAngle = 0;

    const result1 = getAnchorPoint({ x: 0, y: 0, a: 0 }, attachedAngle, surface);
    const result2 = getAnchorPoint({ x: 0, y: 0, a: TAU / 4 }, attachedAngle, surface);

    assert.ok(Math.abs(result2.worldAngle - result1.worldAngle - TAU / 4) < 0.001,
      'world angle should differ by target rotation');
  });

  test('anchor point moves with target position', () => {
    const surface = createCircularSurface(50);
    const attachedAngle = 0;

    const result1 = getAnchorPoint({ x: 0, y: 0, a: 0 }, attachedAngle, surface);
    const result2 = getAnchorPoint({ x: 100, y: 0, a: 0 }, attachedAngle, surface);

    assert.strictEqual(result2.point.x - result1.point.x, 100,
      'anchor point x should shift with target');
  });
});

describe('checkAnchorDeployment', () => {
  test('allows deployment when close and slow', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const surface = createCircularSurface(50);

    const result = checkAnchorDeployment(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      surface
    );

    assert.ok(result.canDeploy, 'should allow deployment');
    assert.ok(isValidNumber(result.localAngle), 'localAngle should be valid');
    assert.ok(isValidNumber(result.distanceToSurface), 'distanceToSurface should be valid');
    assert.ok(isValidNumber(result.relativeSpeed), 'relativeSpeed should be valid');
  });

  test('prevents deployment when too far', () => {
    const vesselPosition = { x: 200, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const surface = createCircularSurface(50);

    const result = checkAnchorDeployment(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      surface
    );

    assert.ok(!result.canDeploy, 'should prevent deployment when too far');
  });

  test('prevents deployment when moving too fast', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 1, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const surface = createCircularSurface(50);

    const result = checkAnchorDeployment(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      surface
    );

    assert.ok(!result.canDeploy, 'should prevent deployment when moving too fast');
  });

  test('returns valid numbers for all outputs', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0.1, y: 0.2, a: 0.01 };
    const targetPosition = { x: 100, y: 0, a: TAU / 3 };
    const targetVelocity = { x: 0.01, y: 0, a: 0.001 };
    const surface = createCircularSurface(50);

    const result = checkAnchorDeployment(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      surface
    );

    assert.ok(isValidNumber(result.localAngle), 'localAngle should be valid');
    assert.ok(isValidNumber(result.distanceToSurface), 'distanceToSurface should be valid');
    assert.ok(isValidNumber(result.relativeSpeed), 'relativeSpeed should be valid');
  });
});

describe('updateAnchoredPhysics', () => {
  test('returns valid state when anchored', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const vesselImpulse = { x: 0, y: 0, a: 0 };
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    assert.ok(isValidPose(result.vesselPosition), 'vesselPosition should be valid');
    assert.ok(isValidVelocity(result.vesselVelocity), 'vesselVelocity should be valid');
    assert.ok(isValidPose(result.targetPosition), 'targetPosition should be valid');
    assert.ok(isValidVelocity(result.targetVelocity), 'targetVelocity should be valid');
  });

  test('returns valid state when vessel is at anchor point', () => {
    const targetPosition = { x: 100, y: 0, a: 0 };
    const surface = createCircularSurface(50);
    // Vessel exactly at anchor point (tetherLength = 0)
    const vesselPosition = { x: 150, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 0, relativeHeading: 0 };
    const vesselImpulse = { x: 0, y: 0, a: 0 };
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    assert.ok(isValidPose(result.vesselPosition), 'vesselPosition should be valid');
    assert.ok(isValidVelocity(result.vesselVelocity), 'vesselVelocity should be valid');
    assert.ok(isValidPose(result.targetPosition), 'targetPosition should be valid');
    assert.ok(isValidVelocity(result.targetVelocity), 'targetVelocity should be valid');
  });

  test('returns valid state with thrust applied', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const vesselImpulse = { x: 0.001, y: 0.001, a: 0.0001 };
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    assert.ok(isValidPose(result.vesselPosition), 'vesselPosition should be valid');
    assert.ok(isValidVelocity(result.vesselVelocity), 'vesselVelocity should be valid');
    assert.ok(isValidPose(result.targetPosition), 'targetPosition should be valid');
    assert.ok(isValidVelocity(result.targetVelocity), 'targetVelocity should be valid');
  });

  test('returns valid state when target is rotating', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: TAU / 4 };
    const targetVelocity = { x: 0, y: 0, a: 0.01 };
    const anchorState = { deployed: true, attachedAngle: TAU / 4, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const vesselImpulse = { x: 0, y: 0, a: 0 };
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    assert.ok(isValidPose(result.vesselPosition), 'vesselPosition should be valid');
    assert.ok(isValidVelocity(result.vesselVelocity), 'vesselVelocity should be valid');
    assert.ok(isValidPose(result.targetPosition), 'targetPosition should be valid');
    assert.ok(isValidVelocity(result.targetVelocity), 'targetVelocity should be valid');
  });

  test('vessel position is derived from asteroid state and tether', () => {
    const targetPosition = { x: 100, y: 0, a: 0 };
    const surface = createCircularSurface(50);
    const tetherLength = 10;
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength, relativeHeading: 0 };
    const vesselPosition = { x: 160, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const vesselImpulse = { x: 0, y: 0, a: 0 };
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    // Vessel should be at: target.x + surfaceRadius + tetherLength = 100 + 50 + 10 = 160
    assert.ok(Math.abs(result.vesselPosition.x - 160) < 0.001, 'vessel x should be at expected position');
    assert.ok(Math.abs(result.vesselPosition.y - 0) < 0.001, 'vessel y should be at expected position');
  });

  test('thrust affects asteroid velocity based on mass', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const vesselImpulse = { x: 0.001, y: 0, a: 0 }; // Thrust in x direction
    const dt = 16;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      TEST_MEAN_RADIUS
    );

    // Asteroid should have gained some velocity in x direction from thrust
    assert.ok(result.targetVelocity.x > 0, 'asteroid should accelerate from thrust');
    assert.ok(isValidNumber(result.targetVelocity.x), 'asteroid velocity should be valid');
  });

  test('larger asteroid accelerates less from same thrust', () => {
    const vesselPosition = { x: 155, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0, y: 0, a: 0 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const vesselImpulse = { x: 0.001, y: 0, a: 0 };
    const dt = 16;

    // Small asteroid (radius 100)
    const resultSmall = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      100
    );

    // Large asteroid (radius 1000)
    const resultLarge = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      vesselImpulse, dt,
      1000
    );

    // Larger asteroid should accelerate less (mass ∝ r³)
    assert.ok(resultSmall.targetVelocity.x > resultLarge.targetVelocity.x,
      'larger asteroid should accelerate less');
  });
});

describe('updateFlightPhysics', () => {
  test('returns valid state', () => {
    const vesselPosition = { x: 0, y: 0, a: 0 };
    const vesselVelocity = { x: 0.1, y: 0.2, a: 0.01 };
    const vesselImpulse = { x: 0.001, y: 0.002, a: 0.0001 };
    const dt = 16;

    const result = updateFlightPhysics(vesselPosition, vesselVelocity, vesselImpulse, dt);

    assert.ok(isValidPose(result.vesselPosition), 'vesselPosition should be valid');
    assert.ok(isValidVelocity(result.vesselVelocity), 'vesselVelocity should be valid');
  });

  test('position changes with velocity', () => {
    const vesselPosition = { x: 0, y: 0, a: 0 };
    const vesselVelocity = { x: 1, y: 0, a: 0 };
    const vesselImpulse = { x: 0, y: 0, a: 0 };
    const dt = 10;

    const result = updateFlightPhysics(vesselPosition, vesselVelocity, vesselImpulse, dt);

    assert.ok(result.vesselPosition.x > 0, 'position should change with velocity');
  });

  test('velocity changes with impulse', () => {
    const vesselPosition = { x: 0, y: 0, a: 0 };
    const vesselVelocity = { x: 0, y: 0, a: 0 };
    const vesselImpulse = { x: 1, y: 0, a: 0 };
    const dt = 10;

    const result = updateFlightPhysics(vesselPosition, vesselVelocity, vesselImpulse, dt);

    assert.ok(result.vesselVelocity.x > 0, 'velocity should change with impulse');
  });
});

describe('updateTargetPosition', () => {
  test('returns valid pose', () => {
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 0.1, y: 0.2, a: 0.01 };
    const dt = 16;

    const result = updateTargetPosition(targetPosition, targetVelocity, dt);

    assert.ok(isValidPose(result), 'result should be valid pose');
  });

  test('position changes with velocity', () => {
    const targetPosition = { x: 100, y: 0, a: 0 };
    const targetVelocity = { x: 1, y: 0, a: 0 };
    const dt = 10;

    const result = updateTargetPosition(targetPosition, targetVelocity, dt);

    assert.strictEqual(result.x, 110, 'x should increase');
  });
});

// Run a simulation for multiple frames to check for NaN accumulation
describe('multi-frame simulation', () => {
  test('anchored physics remains valid over many frames', () => {
    let vesselPosition = { x: 155, y: 0, a: 0 };
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetPosition = { x: 100, y: 0, a: 0 };
    let targetVelocity = { x: 0.001, y: 0, a: 0.0001 };
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 5, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const dt = 16;

    for (let frame = 0; frame < 100; frame++) {
      const vesselImpulse = {
        x: Math.sin(frame * 0.1) * 0.0001,
        y: Math.cos(frame * 0.1) * 0.0001,
        a: 0.00001
      };

      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        vesselImpulse, dt,
        TEST_MEAN_RADIUS
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      assert.ok(isValidPose(vesselPosition), `vesselPosition should be valid at frame ${frame}`);
      assert.ok(isValidVelocity(vesselVelocity), `vesselVelocity should be valid at frame ${frame}`);
      assert.ok(isValidPose(targetPosition), `targetPosition should be valid at frame ${frame}`);
      assert.ok(isValidVelocity(targetVelocity), `targetVelocity should be valid at frame ${frame}`);
    }
  });

  test('anchored physics with rapid asteroid rotation', () => {
    let vesselPosition = { x: 150, y: 0, a: 0 }; // Exactly at surface
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetPosition = { x: 100, y: 0, a: 0 };
    let targetVelocity = { x: 0, y: 0, a: 0.01 }; // Rapid rotation
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 0, relativeHeading: 0 };
    const surface = createCircularSurface(50);
    const dt = 16;

    for (let frame = 0; frame < 200; frame++) {
      const vesselImpulse = { x: 0, y: 0, a: 0 };

      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        vesselImpulse, dt,
        TEST_MEAN_RADIUS
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      assert.ok(isValidPose(vesselPosition), `vesselPosition should be valid at frame ${frame}: ${JSON.stringify(vesselPosition)}`);
      assert.ok(isValidVelocity(vesselVelocity), `vesselVelocity should be valid at frame ${frame}`);
      assert.ok(isValidPose(targetPosition), `targetPosition should be valid at frame ${frame}`);
    }
  });

  test('vessel exactly at anchor point with rotating asteroid', () => {
    // This tests the edge case where vessel is at the anchor point
    let targetPosition = { x: 100, y: 0, a: 0 };
    const surface = createCircularSurface(50);
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 0, relativeHeading: 0 };

    // Vessel at exact anchor point
    let vesselPosition = { x: 150, y: 0, a: 0 };
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetVelocity = { x: 0, y: 0, a: 0.001 };
    const dt = 16;

    for (let frame = 0; frame < 50; frame++) {
      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        { x: 0, y: 0, a: 0 }, dt,
        TEST_MEAN_RADIUS
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      assert.ok(isValidPose(vesselPosition), `vesselPosition should be valid at frame ${frame}: ${JSON.stringify(vesselPosition)}`);
    }
  });

  test('vessel stays pinned to asteroid as it moves', () => {
    const surface = createCircularSurface(50);
    const anchorState = { deployed: true, attachedAngle: 0, tetherLength: 10, relativeHeading: 0 };
    let vesselPosition = { x: 160, y: 0, a: 0 };
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetPosition = { x: 100, y: 0, a: 0 };
    let targetVelocity = { x: 0.1, y: 0.05, a: 0 }; // Asteroid moving
    const dt = 100;

    const result = updateAnchoredPhysics(
      vesselPosition, vesselVelocity,
      targetPosition, targetVelocity,
      anchorState, surface,
      { x: 0, y: 0, a: 0 }, dt,
      TEST_MEAN_RADIUS
    );

    // Vessel should be at fixed offset from new asteroid position
    const expectedVesselX = result.targetPosition.x + 50 + 10; // target.x + radius + tether
    const expectedVesselY = result.targetPosition.y;

    assert.ok(Math.abs(result.vesselPosition.x - expectedVesselX) < 0.001,
      `vessel should be pinned: expected x=${expectedVesselX}, got ${result.vesselPosition.x}`);
    assert.ok(Math.abs(result.vesselPosition.y - expectedVesselY) < 0.001,
      `vessel should be pinned: expected y=${expectedVesselY}, got ${result.vesselPosition.y}`);
  });
});

// Helper to normalize angle to [-PI, PI]
const normalizeAngle = (a) => {
  while (a > Math.PI) a -= TAU;
  while (a < -Math.PI) a += TAU;
  return a;
};

describe('tethered invariants', () => {
  test('relative position preserved in asteroid local frame over multiple frames', () => {
    const surface = createCircularSurface(50);
    const attachedAngle = TAU / 6; // 60 degrees
    const tetherLength = 10;
    const anchorState = { deployed: true, attachedAngle, tetherLength, relativeHeading: 0 };

    // Initial positions
    const initialTargetPosition = { x: 100, y: 50, a: TAU / 4 };
    const surfaceRadius = 50;
    const expectedDistance = surfaceRadius + tetherLength;

    // Compute initial vessel position (should be at attachedAngle from asteroid)
    const initialWorldAngle = attachedAngle + initialTargetPosition.a;
    let vesselPosition = {
      x: initialTargetPosition.x + Math.cos(initialWorldAngle) * expectedDistance,
      y: initialTargetPosition.y + Math.sin(initialWorldAngle) * expectedDistance,
      a: 0
    };
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetPosition = { ...initialTargetPosition };
    let targetVelocity = { x: 0.1, y: 0.05, a: 0.002 }; // Moving and rotating

    // Simulate 100 frames
    for (let frame = 0; frame < 100; frame++) {
      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        { x: 0, y: 0, a: 0 }, 16,
        750
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      // Compute current relative position
      const dx = vesselPosition.x - targetPosition.x;
      const dy = vesselPosition.y - targetPosition.y;
      const currentDistance = Math.hypot(dx, dy);

      // Compute angle in asteroid's local frame
      const worldAngle = Math.atan2(dy, dx);
      const localAngle = normalizeAngle(worldAngle - targetPosition.a);

      // These should remain constant
      assert.ok(Math.abs(currentDistance - expectedDistance) < 0.001,
        `frame ${frame}: distance should be preserved: expected ${expectedDistance}, got ${currentDistance}`);
      assert.ok(Math.abs(normalizeAngle(localAngle - attachedAngle)) < 0.001,
        `frame ${frame}: local angle should be preserved: expected ${attachedAngle.toFixed(4)}, got ${localAngle.toFixed(4)}`);
    }
  });

  test('relative position preserved with thrust applied', () => {
    const surface = createCircularSurface(50);
    const attachedAngle = 0;
    const tetherLength = 15;
    const anchorState = { deployed: true, attachedAngle, tetherLength, relativeHeading: 0 };
    const expectedDistance = 50 + tetherLength;

    let vesselPosition = { x: 165, y: 0, a: 0 };
    let vesselVelocity = { x: 0, y: 0, a: 0 };
    let targetPosition = { x: 100, y: 0, a: 0 };
    let targetVelocity = { x: 0, y: 0, a: 0 };

    // Apply thrust over multiple frames
    for (let frame = 0; frame < 50; frame++) {
      const vesselImpulse = {
        x: Math.sin(frame * 0.2) * 0.0001,
        y: Math.cos(frame * 0.2) * 0.0001,
        a: 0
      };

      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        vesselImpulse, 16,
        750
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      // Compute current relative position
      const dx = vesselPosition.x - targetPosition.x;
      const dy = vesselPosition.y - targetPosition.y;
      const currentDistance = Math.hypot(dx, dy);
      const worldAngle = Math.atan2(dy, dx);
      const localAngle = normalizeAngle(worldAngle - targetPosition.a);

      assert.ok(Math.abs(currentDistance - expectedDistance) < 0.001,
        `frame ${frame}: distance should be preserved under thrust`);
      assert.ok(Math.abs(normalizeAngle(localAngle - attachedAngle)) < 0.001,
        `frame ${frame}: local angle should be preserved under thrust`);
    }
  });

  test('relative position preserved under heavy sustained thrust', () => {
    const surface = createCircularSurface(50);
    const attachedAngle = TAU / 3;
    const tetherLength = 5;
    const anchorState = { deployed: true, attachedAngle, tetherLength, relativeHeading: 0 };
    const expectedDistance = 50 + tetherLength;

    // Start with asteroid already moving
    let targetPosition = { x: 200, y: 100, a: TAU / 8 };
    let targetVelocity = { x: 0.05, y: -0.02, a: 0.001 };

    // Vessel at correct initial position
    const initialWorldAngle = attachedAngle + targetPosition.a;
    let vesselPosition = {
      x: targetPosition.x + Math.cos(initialWorldAngle) * expectedDistance,
      y: targetPosition.y + Math.sin(initialWorldAngle) * expectedDistance,
      a: TAU / 4
    };
    let vesselVelocity = { x: 0, y: 0, a: 0 };

    // Heavy thrust - simulating holding W+A+Q simultaneously
    const vesselImpulse = {
      x: 0.001,   // Forward thrust
      y: 0.0005,  // Lateral thrust
      a: 0.0001   // Rotational thrust (vessel spin, not orbital)
    };

    for (let frame = 0; frame < 200; frame++) {
      const result = updateAnchoredPhysics(
        vesselPosition, vesselVelocity,
        targetPosition, targetVelocity,
        anchorState, surface,
        vesselImpulse, 16,
        750
      );

      vesselPosition = result.vesselPosition;
      vesselVelocity = result.vesselVelocity;
      targetPosition = result.targetPosition;
      targetVelocity = result.targetVelocity;

      // Verify invariants
      const dx = vesselPosition.x - targetPosition.x;
      const dy = vesselPosition.y - targetPosition.y;
      const currentDistance = Math.hypot(dx, dy);
      const worldAngle = Math.atan2(dy, dx);
      const localAngle = normalizeAngle(worldAngle - targetPosition.a);

      assert.ok(Math.abs(currentDistance - expectedDistance) < 0.001,
        `frame ${frame}: distance ${currentDistance.toFixed(4)} should equal ${expectedDistance}`);
      assert.ok(Math.abs(normalizeAngle(localAngle - attachedAngle)) < 0.001,
        `frame ${frame}: local angle ${localAngle.toFixed(4)} should equal ${attachedAngle.toFixed(4)}`);
    }

    // Verify the asteroid actually moved significantly
    assert.ok(targetPosition.x !== 200 || targetPosition.y !== 100,
      'asteroid should have moved from thrust');
    assert.ok(targetVelocity.x !== 0.05 || targetVelocity.y !== -0.02,
      'asteroid velocity should have changed from thrust');
  });
});
