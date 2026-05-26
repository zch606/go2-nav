# Indoor Nav2 Tuning Order

Use this file when the stack exists but behavior is unstable, slow, or unsafe.

## Tune In This Order

1. TF and timestamp sanity
2. Laser liveliness and obstacle representation
3. Odometry smoothness
4. Mapping or localization stability
5. Costmap footprint and inflation
6. Controller limits and goal tolerances
7. Recovery behaviors
8. Waypoint mission repeatability

## Safe Starting Bias

- Prefer lower speeds first
- Prefer shorter test paths first
- Prefer fewer simultaneous parameter changes

## Typical Failure Patterns

- The map looks good, but `odom` drift destabilizes local control
- The footprint is too optimistic for a quadruped stance
- Recoveries look reasonable in logs but cause awkward physical motion
- Controller limits are copied from another robot and exceed what is comfortable indoors
- Obstacle layers are noisy because the sensing pipeline was never cleaned up

## What To Report

When diagnosing issues, summarize:

- What is stable
- What changed most recently
- Whether the failure is global, local, or perception-driven
- What single parameter group should be touched next
