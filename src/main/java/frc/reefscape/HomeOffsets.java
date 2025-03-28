package frc.reefscape;

import frc.robot.Robot;
import lombok.Getter;

/** All offsets in this file are presented in meters */
public class HomeOffsets {

    private static final double halfRobotLength =
            Robot.getSwerve().getConfig().getRobotLength() / 2;

    // red tag offsets
    @Getter private static final double Tag6Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag7Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag8Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag9Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag10Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag11Offset = 1.0 + halfRobotLength;

    // blue tag offsets
    @Getter private static final double Tag17Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag18Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag19Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag20Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag21Offset = 1.0 + halfRobotLength;
    @Getter private static final double Tag22Offset = 1.0 + halfRobotLength;

    // blue tag area goals
    @Getter private final double eventTag17TAGoal = 7.8;
    @Getter private final double eventTag18TAGoal = 7.8;
    @Getter private final double eventTag19TAGoal = 7.8;
    @Getter private final double eventTag20TAGoal = 7.8;
    @Getter private final double eventTag21TAGoal = 7.8;
    @Getter private final double eventTag22TAGoal = 7.8;

    // red tag area goals
    @Getter private final double eventTag6TAGoal = 7.8;
    @Getter private final double eventTag7TAGoal = 7.8;
    @Getter private final double eventTag8TAGoal = 7.8;
    @Getter private final double eventTag9TAGoal = 7.6;
    @Getter private final double eventTag10TAGoal = 7.7;
    @Getter private final double eventTag11TAGoal = 7.8;

    // tag offsets ordered from blue tags to red tags due to centerFaces index values

    @Getter
    private static final double[][] tagOffsets = {
        {17, Tag17Offset},
        {18, Tag18Offset},
        {19, Tag19Offset},
        {20, Tag20Offset},
        {21, Tag21Offset},
        {22, Tag22Offset},
        {6, Tag6Offset},
        {7, Tag7Offset},
        {8, Tag8Offset},
        {9, Tag9Offset},
        {10, Tag10Offset},
        {11, Tag11Offset}
    };
}
