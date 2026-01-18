const TAU = Math.PI * 2;

/**
 * Get the radius of a surface at a given meridian angle.
 * @param {function} describeSurface - Surface description function
 * @param {number} meridian - Angle in radians (0 to TAU)
 * @param {number} T - Threshold for subdivision
 * @returns {number} The radius at that angle
 */
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
    n *= 2;
    d *= 2;
    l += 1;
    description = describeSurface(n + 1, d, before, after, l);
    if (TAU * (n + 1) / d > meridian) {
      after = description.entropy;
    } else {
      before = description.entropy;
      n += 1;
    }
  }
  return description.radius;
};
