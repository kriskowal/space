import { test, describe } from 'node:test';
import assert from 'node:assert';
import { radiusAt } from './surface.js';

const TAU = Math.PI * 2;

// Create a simple surface description function (mimics makeAsteroidSurface)
const createSimpleSurface = (baseRadius) => {
  return (n, d, before, after, l) => ({
    radius: baseRadius,
    entropy: (n / d) * 12345,
  });
};

const isValidNumber = (n) => typeof n === 'number' && isFinite(n) && !isNaN(n);

describe('radiusAt', () => {
  test('returns valid radius for meridian 0', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, 0, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for meridian TAU/4', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, TAU / 4, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for meridian TAU/2', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, TAU / 2, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for meridian TAU', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, TAU, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for meridian slightly over TAU', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, TAU + 0.1, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for negative meridian', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, -0.5, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for large meridian', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    const result = radiusAt(surface, TAU * 3, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius for many random angles', () => {
    const surface = createSimpleSurface(50);
    const T = 5;
    for (let i = 0; i < 100; i++) {
      const meridian = Math.random() * TAU * 2 - TAU;
      const result = radiusAt(surface, meridian, T);
      assert.ok(isValidNumber(result), `result should be valid for meridian ${meridian}, got ${result}`);
    }
  });

  test('returns valid radius with very small T', () => {
    const surface = createSimpleSurface(50);
    const T = 0.1;
    const result = radiusAt(surface, TAU / 4, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });

  test('returns valid radius with T = 0', () => {
    const surface = createSimpleSurface(50);
    const T = 0;
    const result = radiusAt(surface, TAU / 4, T);
    assert.ok(isValidNumber(result), `result should be valid, got ${result}`);
  });
});
