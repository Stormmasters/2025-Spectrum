package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

/** All offsets in this file are presented in meters */
public class HomeOffsets {

    // red tag offsets
    @Getter private static final double Tag6Offset = meterConverter(6);
    @Getter private static final double Tag7Offset = meterConverter(6);
    @Getter private static final double Tag8Offset = meterConverter(6);
    @Getter private static final double Tag9Offset = meterConverter(6);
    @Getter private static final double Tag10Offset = meterConverter(6);
    @Getter private static final double Tag11Offset = meterConverter(6);

    // blue tag offsets
    @Getter private static final double Tag17Offset = meterConverter(6);
    @Getter private static final double Tag18Offset = meterConverter(6);
    @Getter private static final double Tag19Offset = meterConverter(6);
    @Getter private static final double Tag20Offset = meterConverter(6);
    @Getter private static final double Tag21Offset = meterConverter(6);
    @Getter private static final double Tag22Offset = meterConverter(6);

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

    // red tag angles offsets
    // original values: 60, 0, 300, 240, 180, 120
    @Getter private static final double Tag17Angle = radianConverter(60);
    @Getter private static final double Tag18Angle = radianConverter(0);
    @Getter private static final double Tag19Angle = radianConverter(300);
    @Getter private static final double Tag20Angle = radianConverter(240);
    @Getter private static final double Tag21Angle = radianConverter(180);
    @Getter private static final double Tag22Angle = radianConverter(120);

    // blue tag angles offsets
    // original values: 120, 180, 240, 300, 0, 60
    @Getter private static final double Tag6Angle = radianConverter(120);
    @Getter private static final double Tag7Angle = radianConverter(180);
    @Getter private static final double Tag8Angle = radianConverter(240);
    @Getter private static final double Tag9Angle = radianConverter(300);
    @Getter private static final double Tag10Angle = radianConverter(0);
    @Getter private static final double Tag11Angle = radianConverter(60);

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

    /** @param degrees */
    private static double radianConverter(double offsetDegrees) {
        double radianConversion = Units.degreesToRadians(offsetDegrees);

        return radianConversion;
    }

    // tag offsets ordered from blue tags to red tags due to centerFaces index values
    @Getter
    private static final double[][] reefTagOffsets = {
        {17, Tag17Offset, Tag17Angle},
        {18, Tag18Offset, Tag18Angle},
        {19, Tag19Offset, Tag19Angle},
        {20, Tag20Offset, Tag20Angle},
        {21, Tag21Offset, Tag21Angle},
        {22, Tag22Offset, Tag22Angle},
        {6, Tag6Offset, Tag6Angle},
        {7, Tag7Offset, Tag7Angle},
        {8, Tag8Offset, Tag8Angle},
        {9, Tag9Offset, Tag9Angle},
        {10, Tag10Offset, Tag10Angle},
        {11, Tag11Offset, Tag11Angle}
    };
}
