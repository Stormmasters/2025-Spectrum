package frc.reefscape;

import frc.robot.Robot;
import lombok.Getter;

public class WorldsChampsOffsets {

    private static final double halfRobotLength =
            Robot.getSwerve().getConfig().getRobotLength() / 2;

    // red tag offsets
    @Getter private static final double Tag6Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag7Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag8Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag9Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag10Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag11Offset = 0.0 + halfRobotLength;

    // blue tag offsets
    @Getter private static final double Tag17Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag18Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag19Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag20Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag21Offset = 0.0 + halfRobotLength;
    @Getter private static final double Tag22Offset = 0.0 + halfRobotLength;

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
