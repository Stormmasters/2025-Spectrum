package frc.robot.twist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.elevator.ElevatorStates;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class TwistStates {
    private static Twist twist = Robot.getTwist();
    private static TwistConfig config = Robot.getConfig().twist;

    public static void setupDefaultCommand() {
        twist.setDefaultCommand(log(twist.runHoldTwist().withName("Twist.default")));
        // twist.runStop());
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAllStopIntake.and(ElevatorStates.isHome).onTrue(home());
        Robot.getPilot().reZero_start.onTrue(twist.resetToIntialPos());
        stationIntaking.whileTrue(twist.moveToDegrees((config::getStationIntake)));
        L1Coral.whileTrue(twist.moveToDegrees(config::getL1Coral));
        L2Algae.or(L3Algae).whileTrue(twist.moveToDegrees(config::getL2Algae));
        netAlgae.whileTrue(twist.moveToDegrees(config::getBarge));

        branch.whileTrue(rightCoral()); // TODO: Make this flip based on left, right and reversal
    }

    public static DoubleSupplier switchSigns(DoubleSupplier supplier) {
        return () -> -supplier.getAsDouble();
    }

    public static Command runTwist(DoubleSupplier speed) {
        return twist.runPercentage(speed).withName("Twist.runTwist");
    }

    public static Command home() {
        return twist.moveToDegrees(config::getHome).withName("Twist.home");
    }

    public static Command moveToPercentage(DoubleSupplier percent) {
        return twist.moveToPercentage(percent).withName("Twist.moveToPercentage");
    }

    /* Scoring positions */

    public static Command l2Algae() {
        return twist.moveToDegrees(config::getL2Algae).withName("Twist.l2Algae");
    }

    public static Command l3Algae() {
        return twist.moveToDegrees(config::getL3Algae).withName("Twist.l3Algae");
    }

    public static Command leftCoral() {
        return twist.moveToDegrees(() -> twist.checkReversed(config::getLeftCoral))
                .withName("Twist.leftCoral");
    }

    public static Command rightCoral() {
        return twist.moveToDegrees(() -> twist.checkReversed(config::getRightCoral))
                .withName("Twist.rightCoral");
    }

    public static Command floorIntake() {
        return twist.moveToDegrees(config::getClawGroundAlgaeIntake).withName("Twist.floorIntake");
    }

    public static Command coralIntake() {
        return twist.moveToDegrees(() -> twist.checkReversed(config::getStationIntake))
                .withName("Twist.coralIntake");
    }

    public static Command coastMode() {
        return twist.coastMode().withName("Twist.CoastMode");
    }

    public static Command stopMotor() {
        return twist.runStop().withName("Twist.stop");
    }

    public static Command ensureBrakeMode() {
        return twist.ensureBrakeMode().withName("Twist.BrakeMode");
    }

    public static Command barge() {
        return twist.moveToDegrees(() -> twist.checkReversed(config::getBarge))
                .withName("Twist.barge");
    }

    public static Command handOffAlgae() {
        return twist.moveToDegrees(config::getHandAlgae).withName("Twist.handOffAlgae");
    }

    public static Command handOffCoral() {
        return twist.moveToDegrees(config::getHandCoral).withName("Twist.handOffCoral");
    }

    // Tune value command
    public static Command tuneTwist() {
        return twist.moveToDegrees(config::getTuneTwist).withName("Twist.tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    // Negate position command
    protected static Command reverse(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(() -> config.setReversed(true), () -> config.setReversed(false)));
    }
}
