import 'dart:math' as math;

import 'math.dart';

/// Tactical formation geometry shapes supported by [FormationSlotPlanner].
enum FormationShape {
  wedge,
  line,
  circle,
  box,
}

/// Computes local-to-world squad slot offsets aligned to a leader's forward heading.
final class FormationSlotPlanner {
  final FormationShape shape;
  final double spacing;

  const FormationSlotPlanner({
    this.shape = FormationShape.wedge,
    this.spacing = 2.2,
  });

  /// Returns the local-space offset `(right, up, forward)` for slot [slotIndex] in [totalSlots].
  SteerVec3 localOffsetForSlot(int slotIndex, int totalSlots) {
    if (totalSlots <= 1) return SteerVec3.zero;
    switch (shape) {
      case FormationShape.wedge:
        if (slotIndex == 0) return SteerVec3.zero;
        final rank = ((slotIndex - 1) ~/ 2) + 1;
        final side = (slotIndex % 2 == 1) ? -1.0 : 1.0;
        return SteerVec3(side * rank * spacing * 0.85, 0.0, -rank * spacing);

      case FormationShape.line:
        final centered = slotIndex - (totalSlots - 1) * 0.5;
        return SteerVec3(centered * spacing, 0.0, 0.0);

      case FormationShape.circle:
        final radius = math.max(spacing, (spacing * totalSlots) / (2.0 * math.pi));
        final theta = (slotIndex / totalSlots) * math.pi * 2.0;
        return SteerVec3(math.sin(theta) * radius, 0.0, math.cos(theta) * radius);

      case FormationShape.box:
        final cols = math.max(1, math.sqrt(totalSlots).ceil());
        final row = slotIndex ~/ cols;
        final col = slotIndex % cols;
        return SteerVec3(
          (col - (cols - 1) * 0.5) * spacing,
          0.0,
          -row * spacing,
        );
    }
  }

  /// Transforms the local offset for [slotIndex] into world coordinates relative to [leaderPos] and [leaderForward].
  SteerVec3 computeSlotWorldPosition({
    required int slotIndex,
    required int totalSlots,
    required SteerVec3 leaderPos,
    required SteerVec3 leaderForward,
  }) {
    final fwd = leaderForward.lengthSquared > 1e-6
        ? leaderForward.normalized
        : const SteerVec3(0.0, 0.0, -1.0);
    var right = fwd.cross(SteerVec3.unitY);
    if (right.lengthSquared < 1e-6) {
      right = const SteerVec3(1.0, 0.0, 0.0);
    }
    right = right.normalized;
    final up = right.cross(fwd).normalized;

    final local = localOffsetForSlot(slotIndex, totalSlots);
    return leaderPos + right * local.x + up * local.y + fwd * local.z;
  }

  List<SteerVec3> computeAllSlots({
    required int totalSlots,
    required SteerVec3 leaderPos,
    required SteerVec3 leaderForward,
  }) =>
      [
        for (var i = 0; i < totalSlots; i++)
          computeSlotWorldPosition(
            slotIndex: i,
            totalSlots: totalSlots,
            leaderPos: leaderPos,
            leaderForward: leaderForward,
          ),
      ];
}
