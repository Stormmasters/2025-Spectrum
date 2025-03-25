package frc.reefscape;

import frc.robot.Robot;
import lombok.Getter;

public class HomeOffsets {
    // TODO: connect home offsets to Swerve for given offsets
    // TODO: set TA offsets in here for home
    private static final double halfRobotLength =
            Robot.getSwerve().getConfig().getRobotLength() / 2;

    // red tag offsets
    @Getter private static final double Tag6Offset = 0.0;
    @Getter private static final double Tag7Offset = 0.0;
    @Getter private static final double Tag8Offset = 0.0;
    @Getter private static final double Tag9Offset = 0.0;
    @Getter private static final double Tag10Offset = 0.0;
    @Getter private static final double Tag11Offset = 0.0;

    // blue tag offsets
    @Getter private static final double Tag17Offset = 0.0;
    @Getter private static final double Tag18Offset = 0.0;
    @Getter private static final double Tag19Offset = 0.0;
    @Getter private static final double Tag20Offset = 0.0;
    @Getter private static final double Tag21Offset = 0.0;
    @Getter private static final double Tag22Offset = 0.0;

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
        {17, Tag17Offset + halfRobotLength},
        {18, Tag18Offset + halfRobotLength},
        {19, Tag19Offset + halfRobotLength},
        {20, Tag20Offset + halfRobotLength},
        {21, Tag21Offset + halfRobotLength},
        {22, Tag22Offset + halfRobotLength},
        {6, Tag6Offset + halfRobotLength},
        {7, Tag7Offset + halfRobotLength},
        {8, Tag8Offset + halfRobotLength},
        {9, Tag9Offset + halfRobotLength},
        {10, Tag10Offset + halfRobotLength},
        {11, Tag11Offset + halfRobotLength}
    };

    public double getTagOffset(int tag) {
        double homeTag = tag;
        if (homeTag < 0 || homeTag > 22) {
            return -1;
        }

        if (homeTag > 17) {
            homeTag -= 17;
        }

        try {
            return tagOffsets[tag][1];
        } catch (Exception e) {
            return -1;
        }
    }
}
