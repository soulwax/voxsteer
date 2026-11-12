import 'package:test/test.dart';
import 'package:voxsteer/voxsteer.dart';

void main() {
  test('SteeringBehaviors arrive decelerates cleanly and pursue predicts moving target lead', () {
    final seeker = SteeringAgent(id: 1, maxSpeed: 8.0, maxForce: 20.0);
    const target = SteerVec3(0.0, 0.0, 10.0);

    for (var i = 0; i < 120; i++) {
      seeker.addForce(SteeringBehaviors.arrive(seeker, target, slowingRadius: 2.5));
      seeker.step(0.05);
    }
    expect((seeker.position - target).length, lessThan(0.25));
    expect(seeker.speed, lessThan(1.0));

    final pursuer = SteeringAgent(id: 2, position: SteerVec3.zero);
    final leadForce = SteeringBehaviors.pursue(
      pursuer,
      targetPos: const SteerVec3(0.0, 0.0, 6.0),
      targetVelocity: const SteerVec3(4.0, 0.0, 0.0),
    );
    expect(leadForce.x, greaterThan(0.5));
  });

  test('SteeringBehaviors separation & wallAvoidance keep agents separated and clear obstacles', () {
    final a = SteeringAgent(
      id: 1,
      position: const SteerVec3(0.0, 0.0, 0.0),
      velocity: const SteerVec3(0.0, 0.0, 4.0),
      forward: const SteerVec3(0.0, 0.0, 1.0),
    );
    final b = SteeringAgent(
      id: 2,
      position: const SteerVec3(0.5, 0.0, 0.0),
    );

    final sep = SteeringBehaviors.separation(a, [a, b], desiredSeparation: 2.0);
    expect(sep.x, lessThan(-1.0));

    final avoid = SteeringBehaviors.wallAvoidance(
      a,
      const [SteeringObstacle(center: SteerVec3(0.2, 0.0, 2.0), radius: 1.0)],
      feelerLength: 5.0,
    );
    expect(avoid.x.abs(), greaterThan(1.0));
  });

  test('FormationSlotPlanner computes wedge and circle slots relative to leader heading', () {
    const wedge = FormationSlotPlanner(shape: FormationShape.wedge, spacing: 2.0);
    final slots = wedge.computeAllSlots(
      totalSlots: 3,
      leaderPos: SteerVec3.zero,
      leaderForward: const SteerVec3(1.0, 0.0, 0.0),
    );
    expect(slots[0].x, closeTo(0.0, 1e-6));
    // Trailing wingmen sit behind leader (-X) and mirrored symmetrically on Z
    expect(slots[1].x, closeTo(-2.0, 1e-6));
    expect(slots[2].x, closeTo(-2.0, 1e-6));
    expect(slots[1].z, closeTo(-slots[2].z, 1e-6));

    const circle = FormationSlotPlanner(shape: FormationShape.circle, spacing: 3.0);
    final ring = circle.computeAllSlots(
      totalSlots: 4,
      leaderPos: const SteerVec3(10.0, 0.0, 10.0),
      leaderForward: const SteerVec3(0.0, 0.0, -1.0),
    );
    expect((ring[0] - ring[2]).length, greaterThan(5.0));
  });
}
