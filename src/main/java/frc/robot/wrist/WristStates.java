package frc.robot.wrist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.wrist.Wrist.WristConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class WristStates {
    private static Wrist wrist = Robot.getWrist();
    private static WristConfig config = Robot.getConfig().wrist;

    public static void setupDefaultCommand() {
        wrist.setDefaultCommand(
                log(wrist.runHoldWrist().ignoringDisable(true).withName("Wrist.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    public static Command runWrist(DoubleSupplier speed) {
        return wrist.runPercentage(speed).withName("Wrist.runWrist");
    }

    public static Command home() {
        return wrist.moveToPercentage(config::getHome).withName("Wrist.home");
    }

    public static Command moveToPercentage(DoubleSupplier percent) {
        return wrist.moveToPercentage(percent).withName("Wrist.moveToPercentage");
    }

    /* Scoring positions */

    public static Command L2Algae() {
        return wrist.moveToPercentage(config::getL2Algae).withName("Wrist.L2Algae");
    }

    public static Command L3Algae() {
        return wrist.moveToPercentage(config::getL3Algae).withName("Wrist.L3Algae");
    }

    public static Command L2Coral() {
        return wrist.moveToPercentage(config::getL2Coral).withName("Wrist.L2Coral");
    }

    public static Command L3Coral() {
        return wrist.moveToPercentage(config::getL3Coral).withName("Wrist.L3Coral");
    }

    public static Command L1Coral() {
        return wrist.moveToPercentage(config::getL1).withName("Wrist.L1Coral");
    }

    public static Command L4Coral() {
        return wrist.moveToPercentage(config::getL4).withName("Wrist.L4Coral");
    }

    public static Command intake() {
        return wrist.moveToPercentage(config::getIntake).withName("Wrist.intake");
    }

    public static Command coastMode() {
        return wrist.coastMode().withName("Wrist.CoastMode");
    }

    public static Command stopMotor() {
        return wrist.runStop().withName("Wrist.stop");
    }

    public static Command ensureBrakeMode() {
        return wrist.ensureBrakeMode().withName("Wrist.BrakeMode");
    }

    // Tune value command
    public static Command tuneWrist() {
        return wrist.moveToPercentage(config::getTuneWrist).withName("Wrist.tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
