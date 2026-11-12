import 'dart:math' as math;

/// Lightweight immutable 3D vector for `voxsteer`.
final class SteerVec3 {
  final double x;
  final double y;
  final double z;

  const SteerVec3(this.x, this.y, this.z);

  static const zero = SteerVec3(0.0, 0.0, 0.0);
  static const unitY = SteerVec3(0.0, 1.0, 0.0);
  static const unitZ = SteerVec3(0.0, 0.0, 1.0);

  SteerVec3 operator +(SteerVec3 o) => SteerVec3(x + o.x, y + o.y, z + o.z);
  SteerVec3 operator -(SteerVec3 o) => SteerVec3(x - o.x, y - o.y, z - o.z);
  SteerVec3 operator -() => SteerVec3(-x, -y, -z);
  SteerVec3 operator *(double s) => SteerVec3(x * s, y * s, z * s);

  double dot(SteerVec3 o) => x * o.x + y * o.y + z * o.z;

  SteerVec3 cross(SteerVec3 o) => SteerVec3(
        y * o.z - z * o.y,
        z * o.x - x * o.z,
        x * o.y - y * o.x,
      );

  double get lengthSquared => x * x + y * y + z * z;
  double get length => math.sqrt(lengthSquared);

  SteerVec3 get normalized {
    final len = length;
    if (len < 1e-8) return zero;
    return SteerVec3(x / len, y / len, z / len);
  }

  SteerVec3 clampedMagnitude(double maxLen) {
    if (maxLen <= 0.0) return zero;
    final lSq = lengthSquared;
    if (lSq <= maxLen * maxLen) return this;
    final len = math.sqrt(lSq);
    return this * (maxLen / len);
  }
}
