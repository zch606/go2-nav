# Indoor GO2 Navigation Stack

Use this file when designing or reviewing the baseline indoor stack.

## Recommended Shape

1. GO2 state and command bridge
2. Laser or point-cloud pipeline reduced to the representation Nav2 needs
3. Odometry and state estimation layer
4. Mapping or localization layer
5. Nav2 planners, controller, behavior tree, and recoveries

## Mode Split

Keep two explicit modes:

- Mapping mode: build or refine the map
- Localization mode: navigate on a fixed map

Do not blur them in the first deployment. Separate launch and config entry points are easier to debug.

## Frame Priorities

- `base_link` must be stable and match the real body convention
- `odom` must be smooth enough for local control
- `map` must be owned by exactly one localization or SLAM source

## GO2-Specific Reminders

- Use a conservative footprint model for the quadruped stance
- Review lidar mounting height and blind spots instead of copying a wheeled robot setup
- Be skeptical of aggressive spin and backup recovery defaults

## Decision Points

1. Which sensor becomes the primary obstacle source?
2. Which source owns `map -> odom`?
3. Which odometry output feeds local planning?
4. What recoveries are acceptable on a legged platform indoors?
