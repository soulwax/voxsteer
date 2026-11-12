import 'dart:math' as math;

import 'math.dart';

/// Kinematic steering body accumulating weighted forces and integrating bounded velocity & heading.
final class SteeringAgent {
  final int id;
  SteerVec3 position;
  SteerVec3 velocity;
  SteerVec3 forward;

  double mass;
  double maxSpeed;
  double maxForce;
  double maxTurnRateRadPerSec;
  double wanderAngleRad;
  bool planarXz;

  SteerVec3 _accumulatedForce = SteerVec3.zero;
  SteerVec3 _lastAppliedForce = SteerVec3.zero;

  SteeringAgent({
    required this.id,
    this.position = SteerVec3.zero,
    this.velocity = SteerVec3.zero,
    this.forward = const SteerVec3(0.0, 0.0, -1.0),
    this.mass = 1.0,
    this.maxSpeed = 6.0,
    this.maxForce = 18.0,
    this.maxTurnRateRadPerSec = 6.0,
    this.wanderAngleRad = 0.0,
    this.planarXz = true,
  });

  SteerVec3 get accumulatedForce => _accumulatedForce;
  SteerVec3 get lastAppliedForce => _lastAppliedForce;
  double get speed => velocity.length;

  void resetForce() {
    _accumulatedForce = SteerVec3.zero;
  }

  void addForce(SteerVec3 force, {double weight = 1.0}) {
    if (weight <= 0.0) return;
    _accumulatedForce = _accumulatedForce + force * weight;
  }

  /// Integrates accumulated force with maxForce/maxSpeed clamping and damped heading turn.
  void step(double dt) {
    if (dt <= 0.0) return;
    var f = _accumulatedForce;
    if (planarXz) {
      f = SteerVec3(f.x, 0.0, f.z);
    }
    f = f.clampedMagnitude(maxForce);
    _lastAppliedForce = f;
    _accumulatedForce = SteerVec3.zero;

    final safeMass = math.max(0.05, mass);
    final accel = f * (1.0 / safeMass);
    velocity = (velocity + accel * dt).clampedMagnitude(maxSpeed);
    if (planarXz) {
      velocity = SteerVec3(velocity.x, 0.0, velocity.z);
    }
    position = position + velocity * dt;

    final spd = velocity.length;
    if (spd > 1e-4) {
      final desiredFwd = velocity * (1.0 / spd);
      final dot = forward.dot(desiredFwd).clamp(-1.0, 1.0).toDouble();
      final angle = math.acos(dot);
      final maxStep = maxTurnRateRadPerSec * dt;
      if (angle <= maxStep || angle < 1e-5) {
        forward = desiredFwd;
      } else {
        final t = (maxStep / angle).clamp(0.0, 1.0).toDouble();
        forward = (forward * (1.0 - t) + desiredFwd * t).normalized;
      }
    }
  }
}
