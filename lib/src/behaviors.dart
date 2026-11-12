import 'dart:math' as math;

import 'agent.dart';
import 'math.dart';

/// Spherical obstacle volume considered by [SteeringBehaviors.wallAvoidance].
final class SteeringObstacle {
  final SteerVec3 center;
  final double radius;

  const SteeringObstacle({
    required this.center,
    required this.radius,
  });
}

/// Stateless Reynolds steering force evaluators (`seek`, `arrive`, `pursue`, `evade`, `flock`, etc.).
final class SteeringBehaviors {
  const SteeringBehaviors._();

  static SteerVec3 seek(SteeringAgent agent, SteerVec3 target) {
    final toTarget = target - agent.position;
    if (toTarget.lengthSquared < 1e-8) return SteerVec3.zero;
    final desired = toTarget.normalized * agent.maxSpeed;
    return desired - agent.velocity;
  }

  static SteerVec3 flee(
    SteeringAgent agent,
    SteerVec3 threat, {
    double panicDistance = double.infinity,
  }) {
    final away = agent.position - threat;
    final distSq = away.lengthSquared;
    if (distSq > panicDistance * panicDistance) return SteerVec3.zero;
    if (distSq < 1e-8) return agent.forward * agent.maxSpeed - agent.velocity;
    final desired = away.normalized * agent.maxSpeed;
    return desired - agent.velocity;
  }

  static SteerVec3 arrive(
    SteeringAgent agent,
    SteerVec3 target, {
    double slowingRadius = 3.0,
    double stopTolerance = 0.05,
  }) {
    final toTarget = target - agent.position;
    final dist = toTarget.length;
    if (dist <= stopTolerance) {
      return -agent.velocity;
    }
    final safeSlow = math.max(0.1, slowingRadius);
    final rampedSpeed = agent.maxSpeed * (dist / safeSlow);
    final clippedSpeed = math.min(rampedSpeed, agent.maxSpeed);
    final desired = toTarget * (clippedSpeed / dist);
    return desired - agent.velocity;
  }

  static SteerVec3 pursue(
    SteeringAgent agent, {
    required SteerVec3 targetPos,
    required SteerVec3 targetVelocity,
    double maxPredictionSeconds = 1.5,
  }) {
    final toTarget = targetPos - agent.position;
    final speedSum = agent.speed + targetVelocity.length;
    final lookAhead = speedSum < 1e-4
        ? 0.0
        : math.min(maxPredictionSeconds, toTarget.length / speedSum);
    return seek(agent, targetPos + targetVelocity * lookAhead);
  }

  static SteerVec3 evade(
    SteeringAgent agent, {
    required SteerVec3 threatPos,
    required SteerVec3 threatVelocity,
    double maxPredictionSeconds = 1.5,
    double panicDistance = double.infinity,
  }) {
    final toThreat = threatPos - agent.position;
    final speedSum = agent.speed + threatVelocity.length;
    final lookAhead = speedSum < 1e-4
        ? 0.0
        : math.min(maxPredictionSeconds, toThreat.length / speedSum);
    return flee(
      agent,
      threatPos + threatVelocity * lookAhead,
      panicDistance: panicDistance,
    );
  }

  static SteerVec3 wander(
    SteeringAgent agent, {
    required double deltaAngleRad,
    double wanderRadius = 1.5,
    double wanderDistance = 3.0,
  }) {
    agent.wanderAngleRad += deltaAngleRad;
    final circleOffset = SteerVec3(
      math.cos(agent.wanderAngleRad) * wanderRadius,
      0.0,
      math.sin(agent.wanderAngleRad) * wanderRadius,
    );
    final forwardDir = agent.forward.lengthSquared > 1e-6
        ? agent.forward.normalized
        : const SteerVec3(0.0, 0.0, -1.0);
    final targetWorld = agent.position + forwardDir * wanderDistance + circleOffset;
    return seek(agent, targetWorld);
  }

  /// Projects a forward feeler whisker against spherical [obstacles] to steer clear before impact.
  static SteerVec3 wallAvoidance(
    SteeringAgent agent,
    Iterable<SteeringObstacle> obstacles, {
    double feelerLength = 4.0,
    double agentRadius = 0.6,
    double avoidStrength = 25.0,
  }) {
    final dir = agent.velocity.lengthSquared > 1e-4
        ? agent.velocity.normalized
        : agent.forward.normalized;
    if (dir.lengthSquared < 1e-6) return SteerVec3.zero;

    var nearestT = feelerLength;
    SteerVec3 bestSteer = SteerVec3.zero;

    for (final obs in obstacles) {
      final expandedR = obs.radius + agentRadius;
      final toObs = obs.center - agent.position;
      final proj = toObs.dot(dir);
      if (proj < 0.0 || proj > nearestT) continue;
      final closestPt = agent.position + dir * proj;
      final offset = closestPt - obs.center;
      final dist = offset.length;
      if (dist >= expandedR) continue;

      nearestT = proj;
      SteerVec3 lateral;
      if (dist > 1e-4) {
        lateral = offset.normalized;
      } else {
        lateral = SteerVec3(-dir.z, 0.0, dir.x).normalized;
      }
      final urgency = 1.0 - (proj / math.max(0.1, feelerLength));
      bestSteer = lateral * (avoidStrength * (0.5 + urgency));
    }
    return bestSteer;
  }

  static SteerVec3 separation(
    SteeringAgent agent,
    Iterable<SteeringAgent> neighbors, {
    double desiredSeparation = 2.0,
  }) {
    var force = SteerVec3.zero;
    final rSq = desiredSeparation * desiredSeparation;
    for (final n in neighbors) {
      if (n.id == agent.id) continue;
      final delta = agent.position - n.position;
      final dSq = delta.lengthSquared;
      if (dSq > 1e-8 && dSq < rSq) {
        final dist = math.sqrt(dSq);
        force = force + (delta * (1.0 / dist)) * ((desiredSeparation - dist) / desiredSeparation);
      }
    }
    return force.lengthSquared < 1e-8 ? SteerVec3.zero : force * agent.maxForce;
  }

  static SteerVec3 alignment(
    SteeringAgent agent,
    Iterable<SteeringAgent> neighbors, {
    double neighborRadius = 5.0,
  }) {
    var avgHeading = SteerVec3.zero;
    var count = 0;
    final rSq = neighborRadius * neighborRadius;
    for (final n in neighbors) {
      if (n.id == agent.id) continue;
      if ((n.position - agent.position).lengthSquared <= rSq) {
        avgHeading = avgHeading + n.velocity;
        count++;
      }
    }
    if (count == 0 || avgHeading.lengthSquared < 1e-8) return SteerVec3.zero;
    final desired = avgHeading.normalized * agent.maxSpeed;
    return desired - agent.velocity;
  }

  static SteerVec3 cohesion(
    SteeringAgent agent,
    Iterable<SteeringAgent> neighbors, {
    double neighborRadius = 6.0,
  }) {
    var center = SteerVec3.zero;
    var count = 0;
    final rSq = neighborRadius * neighborRadius;
    for (final n in neighbors) {
      if (n.id == agent.id) continue;
      if ((n.position - agent.position).lengthSquared <= rSq) {
        center = center + n.position;
        count++;
      }
    }
    if (count == 0) return SteerVec3.zero;
    return seek(agent, center * (1.0 / count));
  }

  static SteerVec3 flock(
    SteeringAgent agent,
    Iterable<SteeringAgent> neighbors, {
    double separationWeight = 1.6,
    double alignmentWeight = 1.0,
    double cohesionWeight = 1.1,
    double separationRadius = 2.0,
    double neighborRadius = 5.5,
  }) {
    return separation(agent, neighbors, desiredSeparation: separationRadius) * separationWeight +
        alignment(agent, neighbors, neighborRadius: neighborRadius) * alignmentWeight +
        cohesion(agent, neighbors, neighborRadius: neighborRadius) * cohesionWeight;
  }
}
