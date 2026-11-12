<p align="center">
  <img src="assets/logo.svg" width="88" alt="voxsteer logo"/>
</p>

# voxsteer

Pure Dart Reynolds steering behaviors, dynamic flocking, obstacle whiskers, and formation slot planning for `gl_volition`.

## Features
- **`SteeringAgent`**: Velocity/force clamped physics agent with damped angular turn limits and heading alignment.
- **`SteeringBehaviors`**: Stateless force calculators for `seek`, `flee`, `arrive`, `pursue`, `evade`, `wander`, `wallAvoidance`, `separation`, `alignment`, `cohesion`, and combined `flock`.
- **`FormationSlotPlanner`**: Relative-space slot transforms (`wedge`, `line`, `circle`, `box`) rotated around a leader's forward heading.
