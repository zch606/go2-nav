---
name: go2-nav2-humble-skill
description: Indoor GO2 EDU navigation workflow for ROS 2 Humble using Nav2, SLAM Toolbox, and robot_localization on the real robot. Use when Codex needs to design or tune mapping, localization, costmaps, behavior trees, waypoint navigation, launch composition, laser and odometry integration, or diagnose indoor navigation failures on GO2 before expanding outdoors.
---

# GO2 Nav2 Humble Skill

Use this skill for the indoor navigation phase of the GO2 EDU project. It assumes the robot is already reachable and safe, and focuses on building a dependable indoor mapping and navigation stack on ROS 2 Humble.

## Routing

Read only the reference file that fits the current task:

- Indoor stack layout, frame chain, and mode split: `references/stack-architecture.md`
- Parameter strategy, tuning order, and failure diagnosis: `references/nav2-tuning.md`

Pair this skill with:

- `unitree-go2-sdk-skill` when bringup, command ownership, or safety is in scope
- `ros2-engineering-skills` when editing launch files, QoS, tf, or navigation config
- `ros2-skill` when checking live topics, actions, lifecycle nodes, or Nav2 status

## Default Assumptions

- GO2 EDU on ROS 2 Humble
- Real robot, not simulation
- Indoor mapping and navigation is the current target
- The user wants a stable baseline before outdoor expansion

## Core Rules

1. Build the indoor baseline before designing outdoor autonomy.
2. Do not reuse generic wheeled-robot Nav2 params without adapting footprint, speeds, and recoveries.
3. Treat `map -> odom -> base_link` consistency as a hard prerequisite.
4. Start with conservative controller limits and expand only after stable repetition.
5. Keep mapping mode and localization mode distinct, even if they share most launch components.

## Workflow

1. Confirm the robot-side state, odometry, and laser data are trustworthy.
2. Choose the indoor mapping path, then save a known-good map.
3. Choose the indoor localization path for repeat runs.
4. Tune costmaps, footprint padding, inflation, controller limits, and recoveries.
5. Validate waypoint navigation on short, boring paths before longer missions.

## Output Expectations

When using this skill, produce:

- The chosen indoor stack shape
- The highest-risk integration point
- The next tuning step in order
- A short note on what must be revalidated before moving outdoors
