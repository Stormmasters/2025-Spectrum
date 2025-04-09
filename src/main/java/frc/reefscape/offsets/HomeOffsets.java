package frc.reefscape.offsets;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;
import frc.robot.RobotStates;
import lombok.Getter;

/** All offsets in this file are presented in meters */
public class HomeOffsets {

    // blue tag distance offsets, in order of left branch to right branch
    @Getter private static final double[] Tag17Offset = {meterOffset(11.0), meterOffset(11.0)};

    @Getter private static final double[] Tag18Offset = {meterOffset(12.5), meterOffset(12.5)};

    @Getter private static final double[] Tag19Offset = {meterOffset(13.0), meterOffset(13.0)};

    @Getter private static final double[] Tag20Offset = {meterOffset(12.5), meterOffset(12.5)};

    @Getter private static final double[] Tag21Offset = {meterOffset(10.5), meterOffset(10.5)};

    @Getter private static final double[] Tag22Offset = {meterOffset(11.0), meterOffset(11.0)};
    // red tag distance offsets, in order of left branch to right branch
    @Getter private static final double[] Tag6Offset = {meterOffset(11.0), meterOffset(11.0)};
    @Getter private static final double[] Tag7Offset = {meterOffset(12.5), meterOffset(12.5)};
    @Getter private static final double[] Tag8Offset = {meterOffset(13.0), meterOffset(13.0)};
    @Getter private static final double[] Tag9Offset = {meterOffset(12.5), meterOffset(12.5)};

    @Getter private static final double[] Tag10Offset = {meterOffset(10.5), meterOffset(10.5)};

    @Getter private static final double[] Tag11Offset = {meterOffset(11.0), meterOffset(11.0)};

    // blue tag center offsets, in order of left branch to right branch
    @Getter
    private static final double[] Tag17CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag18CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag19CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag20CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag21CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag22CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };
    // red tag center offsets, in order of left branch to right branch
    @Getter
    private static final double[] Tag6CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag7CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag8CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag9CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag10CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    @Getter
    private static final double[] Tag11CenterOffset = {
        Units.inchesToMeters(0.0), Units.inchesToMeters(0.0)
    };

    // blue tag area goals
    // may be deprecated if not used
    @Getter private static final double Tag17TAGoal = 7.9; // 7.8
    @Getter private static final double Tag18TAGoal = 7.9; // 7.8
    @Getter private static final double Tag19TAGoal = 7.9; // 7.8
    @Getter private static final double Tag20TAGoal = 7.9; // 7.8
    @Getter private static final double Tag21TAGoal = 7.9; // 7.8
    @Getter private static final double Tag22TAGoal = 7.9; // 7.8

    // red tag area goals
    // may be deprecated if not used
    @Getter private static final double Tag6TAGoal = 7.9; // 7.8;
    @Getter private static final double Tag7TAGoal = 7.9; // 7.8;
    @Getter private static final double Tag8TAGoal = 7.9; // 7.8;
    @Getter private static final double Tag9TAGoal = 7.9; // 7.6;
    @Getter private static final double Tag10TAGoal = 7.9; // 7.7;
    @Getter private static final double Tag11TAGoal = 7.9; // 7.8;

    // red tag angles offsets
    // original values: all 180
    @Getter private static final double Tag17Angle = radianConverter(180);
    @Getter private static final double Tag18Angle = radianConverter(180);
    @Getter private static final double Tag19Angle = radianConverter(180);
    @Getter private static final double Tag20Angle = radianConverter(180);
    @Getter private static final double Tag21Angle = radianConverter(180);
    @Getter private static final double Tag22Angle = radianConverter(180);

    // blue tag angles offsets
    // original values: all 180
    @Getter private static final double Tag6Angle = radianConverter(180);
    @Getter private static final double Tag7Angle = radianConverter(180);
    @Getter private static final double Tag8Angle = radianConverter(180);
    @Getter private static final double Tag9Angle = radianConverter(180);
    @Getter private static final double Tag10Angle = radianConverter(180);
    @Getter private static final double Tag11Angle = radianConverter(180);

    /**
     * Converts inches to meters and adds half the robot length to the offset
     *
     * @param offsetInches
     * @return
     */
    private static double meterOffset(double offsetInches) {
        double meterConversion = Units.inchesToMeters(offsetInches);

        return addRobotLength(meterConversion);
    }

    /**
     * Adds robot length to the given offset meters
     *
     * @param offsetMeters
     * @return
     */
    private static double addRobotLength(double offsetMeters) {
        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;
        return halfRobotLength + offsetMeters;
    }

    /** @param degrees */
    private static double radianConverter(double offsetDegrees) {
        double radianConversion = Units.degreesToRadians(offsetDegrees);

        return radianConversion;
    }

    // tag offsets ordered from blue tags to red tags due to centerFaces index values
    @Getter
    private static final double[][] reefTagOffsets = {
        {
            17,
            Tag17Offset[0],
            Tag17Offset[1],
            Tag17CenterOffset[0],
            Tag17CenterOffset[1],
            Tag17Angle
        },
        {
            18,
            Tag18Offset[0],
            Tag18Offset[1],
            Tag18CenterOffset[0],
            Tag18CenterOffset[1],
            Tag18Angle
        },
        {
            19,
            Tag19Offset[0],
            Tag19Offset[1],
            Tag19CenterOffset[0],
            Tag19CenterOffset[1],
            Tag19Angle
        },
        {
            20,
            Tag20Offset[0],
            Tag20Offset[1],
            Tag20CenterOffset[0],
            Tag20CenterOffset[1],
            Tag20Angle
        },
        {
            21,
            Tag21Offset[0],
            Tag21Offset[1],
            Tag21CenterOffset[0],
            Tag21CenterOffset[1],
            Tag21Angle
        },
        {
            22,
            Tag22Offset[0],
            Tag22Offset[1],
            Tag22CenterOffset[0],
            Tag22CenterOffset[1],
            Tag22Angle
        },
        {6, Tag6Offset[0], Tag6Offset[1], Tag6CenterOffset[0], Tag6CenterOffset[1], Tag6Angle},
        {7, Tag7Offset[0], Tag7Offset[1], Tag7CenterOffset[0], Tag7CenterOffset[1], Tag7Angle},
        {8, Tag8Offset[0], Tag8Offset[1], Tag8CenterOffset[0], Tag8CenterOffset[1], Tag8Angle},
        {9, Tag9Offset[0], Tag9Offset[1], Tag9CenterOffset[0], Tag9CenterOffset[1], Tag9Angle},
        {
            10,
            Tag10Offset[0],
            Tag10Offset[1],
            Tag10CenterOffset[0],
            Tag10CenterOffset[1],
            Tag10Angle
        },
        {11, Tag11Offset[0], Tag11Offset[1], Tag11CenterOffset[0], Tag11CenterOffset[1], Tag11Angle}
    };

    @Getter
    private static double[][] tagAreaOffsets = {
        {17, Tag17TAGoal},
        {18, Tag18TAGoal},
        {19, Tag19TAGoal},
        {20, Tag20TAGoal},
        {21, Tag21TAGoal},
        {22, Tag22TAGoal},
        {6, Tag6TAGoal},
        {7, Tag7TAGoal},
        {8, Tag8TAGoal},
        {9, Tag9TAGoal},
        {10, Tag10TAGoal},
        {11, Tag11TAGoal}
    };

    // ---------------------------------------------------------------
    // Helper Methods
    // ---------------------------------------------------------------

    public static int getReefTagIDToOffsetIndex(int tagID) {
        if (tagID < 6 || tagID == 16 || tagID > 22) {
            return -1;
        }

        int offsetIndex = tagID;

        if (offsetIndex >= 17) {
            offsetIndex -= 17;
        }

        return offsetIndex;
    }

    public static double getReefTagDistanceOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }
        if (RobotStates.rightScore.getAsBoolean()) {
            return reefTagOffsets[offsetIndex][2];
        }
        return reefTagOffsets[offsetIndex][1];
    }

    public static double getReefTagCenterOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }
        if (RobotStates.rightScore.getAsBoolean()) {
            return reefTagOffsets[offsetIndex][4];
        }
        return reefTagOffsets[offsetIndex][3];
    }

    public static double getReefTagAngleOffset(int tagID) {
        int offsetIndex = getReefTagIDToOffsetIndex(tagID);
        if (offsetIndex == -1) {
            return 0.0;
        }
        return reefTagOffsets[offsetIndex][5];
    }
}
