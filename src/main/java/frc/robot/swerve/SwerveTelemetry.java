package frc.robot.swerve;

import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SuppliedValueWidget;
import java.util.Map;

public class SwerveTelemetry {
    protected ShuffleboardTab tab;
    private Swerve swerve;
    boolean initialized = false;
    private Odometry odometry;

    public SwerveTelemetry(Swerve swerve) {
        this.swerve = swerve;
        tab = Shuffleboard.getTab("Swerve");
        tab.addNumber("Heading", () -> swerve.getRotation().getDegrees()).withPosition(0, 0);
        tab.addNumber("Odometry X", () -> swerve.getRobotPose().getX()).withPosition(0, 1);
        tab.addNumber("Odometry Y", () -> swerve.getRobotPose().getY()).withPosition(0, 2);
        tab.addNumber("Odometry X2", () -> odometry.getPoseMeters().getX()).withPosition(0, 3);
        tab.addNumber("Odometry Y2", () -> odometry.getPoseMeters().getY()).withPosition(0, 4);
        // tab.addNumber("Odometry X", () -> swerve.getPoseMeters().getX()).withPosition(0, 1);
        // tab.addNumber("Odometry Y", () -> swerve.getPoseMeters().getY()).withPosition(0, 2);
        // tab.addNumber("Angle", () -> swerve.gyro.getRawAngle().getDegrees()).withPosition(2, 0);
        // tab.addNumber("Yaw", () -> swerve.gyro.getRawYaw().getDegrees()).withPosition(2, 1);
        // tab.addNumber("Roll", () -> swerve.gyro.getRawRoll().getDegrees()).withPosition(2, 2);
        // tab.addNumber("Roll Rate", () -> swerve.gyro.getRollRate()).withPosition(2, 4);
        // tab.addNumber("Pitch", () -> swerve.gyro.getRawPitch().getDegrees()).withPosition(2, 3);
    }

    public void testMode() {
        if (!initialized) {
            moduleLayout("Mod 0", 0, tab).withPosition(1, 0);
            moduleLayout("Mod 1", 1, tab).withPosition(2, 0);
            moduleLayout("Mod 2", 2, tab).withPosition(3, 0);
            moduleLayout("Mod 3", 3, tab).withPosition(4, 0);
            initialized = true;
        }
    }

    // TODO: implement logging
    // public void logModuleStates(String key, SwerveModuleState[] states) {
    //     Robot.log.logger.recordOutput(key, states);
    // }

    // public void logModuleAbsolutePositions() {
    //     for (SwerveModule mod : swerve.mSwerveMods) {
    //         Robot.log.logger.recordOutput(
    //                 "Mod " + mod.moduleNumber + " Absolute",
    // mod.getAbsoluteAngle().getDegrees());
    //     }
    // }

    public ShuffleboardLayout moduleLayout(String name, int moduleNum, ShuffleboardTab tab) {
        ShuffleboardLayout modLayout = tab.getLayout(name, BuiltInLayouts.kGrid);
        // m_mod0Layout.withSize(1, 2);
        modLayout.withProperties(Map.of("Label position", "TOP"));

        // mod0 Cancoder Angle
        SuppliedValueWidget<Double> mod0CancoderAngleWidget =
                modLayout.addNumber(
                        "Cancoder Angle",
                        () ->
                                swerve.getModules()[moduleNum]
                                        .getCachedPosition()
                                        .angle
                                        .getDegrees());
        mod0CancoderAngleWidget.withPosition(0, 0);

        // mod0 Integrated Angle
        SuppliedValueWidget<Double> mod0IntegratedAngleWidget =
                modLayout.addNumber(
                        "Integrated Angle",
                        () -> swerve.getModules()[moduleNum].getCurrentState().angle.getDegrees());
        mod0IntegratedAngleWidget.withPosition(0, 1);

        // mod0 Velocity
        SuppliedValueWidget<Double> mod0VelocityWidget =
                modLayout.addNumber(
                        "Wheel Velocity",
                        () ->
                                swerve.getModules()[moduleNum].getCurrentState()
                                        .speedMetersPerSecond);
        mod0VelocityWidget.withPosition(0, 2);
        return modLayout;
    }
}
