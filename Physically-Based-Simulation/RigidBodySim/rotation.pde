class Vec3 extends PVector {
  /* ... */
  PVector rotate(float angle, PVector axis) {
    return rotate(angle, axis.x, axis.y, axis.z);
  }

  PVector rotate(float angle, float axisx, float axisy, float axisz) {
    // Find squares of each axis component.
    float xsq = axisx * axisx;
    float ysq = axisy * axisy;
    float zsq = axisz * axisz;

    // Test the axis's magnitude.
    float mag = xsq + ysq + zsq;

     // EPSILON is the smallest positive non-zero amount.
    // Return if the axis has no length or the point is < 0, 0, 0 >.
    if (mag < EPSILON || (x == 0.0 && y == 0.0 && z == 0.0)) {
      return this;
    } else if (mag > 1.0) {
      mag = 1.0 / sqrt(mag);
      axisx *= mag; axisy *= mag; axisz *= mag;
      xsq = axisx * axisx; ysq = axisy * axisy; zsq = axisz * axisz;
    }

    float cosa = cos(angle);
    float sina = sin(angle);
    float complcos = 1.0 - cosa;

    float complxy = complcos * axisx * axisy;
    float complxz = complcos * axisx * axisz;
    float complyz = complcos * axisy * axisz;

    float sinx = sina * axisx;
    float siny = sina * axisy;
    float sinz = sina * axisz;

    // Right on the x axis (i).
    float ix = complcos * xsq + cosa; /* m00 */
    float iy = complxy + sinz; /* m10 */
    float iz = complxz - siny; /* m20 */

    // Up on the y axis (j).
    float jx = complxy - sinz; /* m01 */
    float jy = complcos * ysq + cosa; /* m11 */
    float jz = complyz + sinx; /* m21 */

    // Forward on the z axis (k).
    float kx = complxz + siny; /* m02 */
    float ky = complyz - sinx; /* m12 */
    float kz = complcos * zsq + cosa; /* m22 */

    float tempx = x; float tempy = y;
    x = ix * x + jx * y + kx * z;
    y = iy * tempx + jy * y + ky * z;
    z = iz * tempx + jz * tempy + kz * z;

    return this;
  }
}