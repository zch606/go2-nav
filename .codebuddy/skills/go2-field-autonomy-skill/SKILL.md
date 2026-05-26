---
name: go2-field-autonomy-skill
description: Outdoor GO2 EDU autonomy workflow for ROS 2 Humble after the indoor baseline is stable. Use when Codex needs to plan or debug waypoint missions, terrain-aware routing, outdoor localization changes, traversability checks, safety envelopes, mission logging, or autonomy_stack_go2-style field navigation on the real robot.
---

# GO2 Field Autonomy Skill

Use this skill after the indoor stack works consistently. It focuses on the transition from lab-grade navigation to outdoor waypoint autonomy, where sensing, localization, and safety assumptions all change.

## Routing

Read only the reference file that matches the task:

- Architecture shift from indoor Nav2 to outdoor field autonomy: `references/outdoor-transition.md`
- Mission rehearsal, logging, and field validation: `references/mission-validation.md`

Pair this skill with:

- `go2-nav2-humble-skill` to preserve the indoor baseline while expanding outward
- `unitree-go2-sdk-skill` when safety, bringup, or command ownership changes
- `ros2-skill` for live field checks and waypoint execution

## Default Assumptions

- GO2 EDU on ROS 2 Humble
- Outdoor work starts only after indoor navigation is repeatable
- Real robot tests need stronger validation than lab tests

## Core Rules

1. Treat outdoor work as a new operating regime, not just a new parameter file.
2. Keep the indoor baseline preserved while outdoor branches evolve.
3. Re-justify localization, obstacle sensing, and planner behavior for outdoor terrain.
4. Prefer short waypoint loops and bagged validation before long missions.
5. Raise mission complexity only after route repeatability is demonstrated.

## Workflow

1. Identify what changes between indoor and outdoor sensing.
2. Decide whether plain Nav2 is enough or a field-oriented stack is needed.
3. Rebuild the localization and planning assumptions for outdoor conditions.
4. Rehearse with short waypoint missions and strong logging.
5. Expand mission length only after stable repetition and safety review.

## Output Expectations

When using this skill, produce:

- The outdoor architecture delta from the indoor baseline
- The most likely failure mode in the next field test
- The safest next mission shape
- The minimal validation evidence required before scaling up
