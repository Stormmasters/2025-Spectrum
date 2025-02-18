package frc.robot.twist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Robot;
import frc.robot.twist.Twist.TwistConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class TwistStates {
    private static Twist twist = Robot.getTwist();
    private static TwistConfig config = Robot.getConfig().twist;

    public static void setupDefaultCommand() {
        twist.setDefaultCommand(
                log(twist.runHoldTwist().ignoringDisable(true).withName("Twist.default")));
    }

    public static void setStates() {
        stationIntaking.and(backwardMode.not()).whileTrue(log(coralIntake()));
        stationIntaking.and(backwardMode).whileTrue(log(reverse(coralIntake())));
        stationExtendedIntake.and(backwardMode.not()).whileTrue(log(coralIntake()));
        stationExtendedIntake.and(backwardMode).whileTrue(log(reverse(coralIntake())));

        L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));

        coralReefPosition.and(backwardMode.not()).and(leftScore).whileTrue(log(leftCoral()));
        coralReefPosition.and(backwardMode).and(leftScore).whileTrue(log(reverse(leftCoral())));
        coralReefPosition.and(backwardMode.not()).and(rightScore).whileTrue(log(rightCoral()));
        coralReefPosition.and(backwardMode).and(rightScore).whileTrue(log(reverse(rightCoral())));

        barge.and(backwardMode.not()).whileTrue(log(barge()));
        barge.and(backwardMode).whileTrue(log(reverse(barge())));

        homeAll.whileTrue(log(home()));

        algaeHandoff.whileTrue(log(handOffAlgae()));
        coralHandoff.whileTrue(log(handOffCoral()));

        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    public static DoubleSupplier switchSigns(DoubleSupplier supplier) {
        return () -> -supplier.getAsDouble();
    }

    public static Command runTwist(DoubleSupplier speed) {
        return twist.runPercentage(speed).withName("Twist.runTwist");
    }

    public static Command home() {
        return twist.moveToPercentage(config::getHome).withName("Twist.home");
    }

    public static Command moveToPercentage(DoubleSupplier percent) {
        return twist.moveToPercentage(percent).withName("Twist.moveToPercentage");
    }

    /* Scoring positions */

    public static Command l2Algae() {
        return twist.moveToPercentage(config::getL2Algae).withName("Twist.l2Algae");
    }

    public static Command l3Algae() {
        return twist.moveToPercentage(config::getL3Algae).withName("Twist.l3Algae");
    }

    public static Command leftCoral() {
        return twist.moveToPercentage(() -> twist.checkReversed(config::getLeftCoral))
                .withName("Twist.leftCoral");
    }

    public static Command rightCoral() {
        return twist.moveToPercentage(() -> twist.checkReversed(config::getRightCoral))
                .withName("Twist.rightCoral");
    }

    public static Command floorIntake() {
        return twist.moveToPercentage(config::getClawGroundAlgaeIntake)
                .withName("Twist.floorIntake");
    }

    public static Command coralIntake() {
        return twist.moveToPercentage(() -> twist.checkReversed(config::getCoralIntake))
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
        return twist.moveToPercentage(() -> twist.checkReversed(config::getBarge))
                .withName("Twist.barge");
    }

    public static Command handOffAlgae() {
        return twist.moveToPercentage(config::getHandAlgae).withName("Twist.handOffAlgae");
    }

    public static Command handOffCoral() {
        return twist.moveToPercentage(config::getHandCoral).withName("Twist.handOffCoral");
    }

    // Tune value command
    public static Command tuneTwist() {
        return twist.moveToPercentage(config::getTuneTwist).withName("Twist.tune");
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
