package frc.reefscape.offsets;

import edu.wpi.first.math.util.Units;
import frc.robot.Robot;

public class TagProperties {
    private final double[] offset;
    private final double[] centerOffset;
    private final double taGoal;
    private final double angle;

    public TagProperties(double[] offset, double[] centerOffset, double taGoal, double angle) {
        this.offset = offset;
        this.centerOffset = centerOffset;
        this.taGoal = taGoal;
        this.angle = angle;
    }

    public double[] getOffset() {
        return offset;
    }

    public double[] getCenterOffset() {
        return centerOffset;
    }

    public double getTaGoal() {
        return taGoal;
    }

    public double getAngle() {
        return angle;
    }

    private static double meterOffset(double offsetInches) {
        double meterConversion = Units.inchesToMeters(offsetInches);
        double halfRobotLength = Robot.getSwerve().getConfig().getRobotLength() / 2;
        return meterConversion + halfRobotLength;
    }

    private static double radianConverter(double offsetDegrees) {
        return Units.degreesToRadians(offsetDegrees);
    }
}
