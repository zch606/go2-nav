# Field Mission Validation

Use this file before and after outdoor tests.

## Pre-Mission

1. Define the shortest useful waypoint loop.
2. Confirm estop access, line of sight, and operator roles.
3. Decide what must be bagged and what success looks like.
4. Confirm the robot can fall back to a smaller safe mode if the mission degrades.

## During Mission

- Watch for localization confidence collapse
- Watch for terrain-driven planner indecision
- Watch for behaviors that were acceptable indoors but unsafe outdoors

## After Mission

Summarize:

- Did the route repeat?
- Where did localization degrade?
- Was obstacle handling conservative enough?
- Which one subsystem should be changed next?

## Scale-Up Rule

Increase only one of these at a time:

- Mission distance
- Mission speed
- Terrain difficulty
- Autonomy complexity
