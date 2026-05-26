from dataclasses import dataclass
from math import fabs


class MotionGait:
    STAND = 0
    WALK = 1
    TROT = 2
    CRAWL = 3


class SafetyLevel:
    NORMAL = 0
    DEGRADED = 1
    SLOW = 2
    PAUSE = 3
    STOP = 4
    MANUAL = 5


@dataclass(frozen=True)
class SafetyAssessment:
    state: int
    severity: int
    emergency_stop: bool
    allow_motion: bool
    speed_scale: float
    reason: str


@dataclass(frozen=True)
class MotionProfile:
    gait: int
    body_height: float
    duration: float
    speed_limit: float


def evaluate_safety(
    roll_deg: float,
    pitch_deg: float,
    terrain_cost: float,
    odom_covariance: float,
    cmd_linear_speed: float,
    cmd_angular_speed: float,
) -> SafetyAssessment:
    max_tilt = max(fabs(roll_deg), fabs(pitch_deg))

    if max_tilt >= 20.0 or terrain_cost >= 220.0 or odom_covariance >= 0.8:
        return SafetyAssessment(
            state=SafetyLevel.STOP,
            severity=4,
            emergency_stop=True,
            allow_motion=False,
            speed_scale=0.0,
            reason='critical tilt, terrain, or localization risk',
        )

    if max_tilt >= 14.0 or terrain_cost >= 180.0 or odom_covariance >= 0.45:
        return SafetyAssessment(
            state=SafetyLevel.PAUSE,
            severity=3,
            emergency_stop=False,
            allow_motion=False,
            speed_scale=0.0,
            reason='high terrain or localization risk',
        )

    if (
        max_tilt >= 8.0
        or terrain_cost >= 130.0
        or odom_covariance >= 0.25
        or fabs(cmd_angular_speed) >= 0.8
        or fabs(cmd_linear_speed) >= 0.6
    ):
        return SafetyAssessment(
            state=SafetyLevel.SLOW,
            severity=2,
            emergency_stop=False,
            allow_motion=True,
            speed_scale=0.5,
            reason='moderate terrain or tracking risk',
        )

    return SafetyAssessment(
        state=SafetyLevel.NORMAL,
        severity=0,
        emergency_stop=False,
        allow_motion=True,
        speed_scale=1.0,
        reason='nominal',
    )


def select_motion_profile(linear_speed: float, angular_speed: float, safety_state: int, speed_scale: float) -> MotionProfile:
    effective_speed = fabs(linear_speed) * max(0.0, speed_scale)
    effective_turn = fabs(angular_speed)

    if safety_state in (SafetyLevel.STOP, SafetyLevel.PAUSE, SafetyLevel.MANUAL):
        return MotionProfile(
            gait=MotionGait.STAND,
            body_height=0.26,
            duration=0.10,
            speed_limit=0.0,
        )

    if effective_speed < 0.05 and effective_turn < 0.12:
        return MotionProfile(
            gait=MotionGait.STAND,
            body_height=0.25,
            duration=0.10,
            speed_limit=0.20,
        )

    if effective_speed < 0.20:
        return MotionProfile(
            gait=MotionGait.WALK,
            body_height=0.23,
            duration=0.12,
            speed_limit=0.35,
        )

    if effective_turn > 0.45:
        return MotionProfile(
            gait=MotionGait.CRAWL,
            body_height=0.24,
            duration=0.14,
            speed_limit=0.30,
        )

    return MotionProfile(
        gait=MotionGait.TROT,
        body_height=0.21,
        duration=0.10,
        speed_limit=0.60,
    )
