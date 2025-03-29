package frc.reefscape;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import lombok.Getter;

public class StateChampsOffsets {

    // red tag offsets
    @Getter private static final double Tag6Offset = offsetInMeters(24);
    @Getter private static final double Tag7Offset = offsetInMeters(24);
    @Getter private static final double Tag8Offset = offsetInMeters(24);
    @Getter private static final double Tag9Offset = offsetInMeters(24);
    @Getter private static final double Tag10Offset = offsetInMeters(24);
    @Getter private static final double Tag11Offset = offsetInMeters(24);

    // blue tag offsets
    @Getter private static final double Tag17Offset = offsetInMeters(24);
    @Getter private static final double Tag18Offset = offsetInMeters(24);
    @Getter private static final double Tag19Offset = offsetInMeters(24);
    @Getter private static final double Tag20Offset = offsetInMeters(24);
    @Getter private static final double Tag21Offset = offsetInMeters(24);
    @Getter private static final double Tag22Offset = offsetInMeters(24);

    /**
     * Converts inches to meters and adds half the robot length to the offset
     *
     * @param offsetInches
     * @return
     */
    private static double offsetInMeters(double offsetInches) {

        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;

        double meterConversion = Units.inchesToMeters(offsetInches) + halfRobotLength;

        return meterConversion;
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
}
