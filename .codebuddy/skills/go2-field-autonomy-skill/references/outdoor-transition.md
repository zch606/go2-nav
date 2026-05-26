# Outdoor Transition Plan

Use this file when the team is moving from indoor navigation to outdoor autonomy.

## What Usually Changes

- Localization source or fusion strategy
- Obstacle representation and sensing range
- Planner horizon and route structure
- Mission validation discipline

## Decision Frame

Ask these questions:

1. Is the outdoor task still "plan around obstacles on a known map", or is it now "follow waypoints through changing terrain"?
2. Is the current odometry and localization source reliable outside?
3. Do we need traversability reasoning instead of only occupancy reasoning?
4. Can the current recovery behaviors remain enabled?

## When Plain Nav2 May Still Be Enough

- Outdoor space is structured and relatively flat
- Missions are short and line of sight is maintained
- The sensor suite already supports reliable localization

## When A Field-Oriented Stack Is More Appropriate

- Terrain quality changes the feasible route
- Traversability matters more than geometric free space alone
- The mission is waypoint-centric and longer-range
- The team is evaluating autonomy_stack_go2-like behavior
