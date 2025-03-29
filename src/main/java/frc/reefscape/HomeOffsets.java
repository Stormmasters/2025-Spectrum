package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

/** All offsets in this file are presented in meters */
public class HomeOffsets {

    // red tag offsets
    @Getter private static final double Tag6Offset = meterConverter(24);
    @Getter private static final double Tag7Offset = meterConverter(24);
    @Getter private static final double Tag8Offset = meterConverter(24);
    @Getter private static final double Tag9Offset = meterConverter(24);
    @Getter private static final double Tag10Offset = meterConverter(24);
    @Getter private static final double Tag11Offset = meterConverter(24);

    // blue tag offsets
    @Getter private static final double Tag17Offset = meterConverter(24);
    @Getter private static final double Tag18Offset = meterConverter(24);
    @Getter private static final double Tag19Offset = meterConverter(24);
    @Getter private static final double Tag20Offset = meterConverter(24);
    @Getter private static final double Tag21Offset = meterConverter(24);
    @Getter private static final double Tag22Offset = meterConverter(24);

    // blue tag area goals
    // may be deprecated if not used
    @Getter private static final double eventTag17TAGoal = 7.8;
    @Getter private static final double eventTag18TAGoal = 7.8;
    @Getter private static final double eventTag19TAGoal = 7.8;
    @Getter private static final double eventTag20TAGoal = 7.8;
    @Getter private static final double eventTag21TAGoal = 7.8;
    @Getter private static final double eventTag22TAGoal = 7.8;

    // red tag area goals
    // may be deprecated if not used
    @Getter private static final double eventTag6TAGoal = 7.8;
    @Getter private static final double eventTag7TAGoal = 7.8;
    @Getter private static final double eventTag8TAGoal = 7.8;
    @Getter private static final double eventTag9TAGoal = 7.6;
    @Getter private static final double eventTag10TAGoal = 7.7;
    @Getter private static final double eventTag11TAGoal = 7.8;

    /**
     * Converts inches to meters and adds half the robot length to the offset
     *
     * @param offsetInches
     * @return
     */
    private static double meterConverter(double offsetInches) {
        double meterConversion = Units.inchesToMeters(offsetInches);
        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;

        return meterConversion + halfRobotLength;
    }

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

    @Getter
    private static final double[][] tagAreaOffsets = {
        {17, eventTag17TAGoal},
        {18, eventTag18TAGoal},
        {19, eventTag19TAGoal},
        {20, eventTag20TAGoal},
        {21, eventTag21TAGoal},
        {22, eventTag22TAGoal},
        {6, eventTag6TAGoal},
        {7, eventTag7TAGoal},
        {8, eventTag8TAGoal},
        {9, eventTag9TAGoal},
        {10, eventTag10TAGoal},
        {11, eventTag11TAGoal}
    };
}
