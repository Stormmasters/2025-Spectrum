package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static final Trigger isHome =
            elbow.atDegrees(() -> (config.getHome() + config.getOffset()), config::getTolerance);
    public static final Trigger pastElevator =
            elbow.aboveDegrees(elbow.offsetPosition(() -> -160 + 360), config::getTolerance)
                    .and(elbow.belowDegrees(() -> 160, config::getTolerance));
    public static final Trigger scoreL4 =
            elbow.atDegrees(() -> (config.getL4Coral() - 12), config::getTolerance);

    public static final Trigger atTarget = elbow.atTargetPosition(() -> 0.001);

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(log(elbow.runHoldElbow().withName("Elbow.default")));
        // elbow.runStop());
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        homeAll.whileTrue(home());

        stationIntaking
                .and(backwardMode.not(), actionState.not())
                .whileTrue(elbow.moveToDegrees(config::getStationIntake));

        // stages elbow
        stagedCoral
                .and(backwardMode.not(), actionState.not())
                .whileTrue(elbow.moveToDegrees(config::getStage));
        stagedCoral
                .and(backwardMode, actionState.not())
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getStage())); // TODO: change back to command

        L1Coral.and(backwardMode.not(), actionPrepState).whileTrue(l1Coral());

        L2Coral.and(backwardMode.not(), actionPrepState, ElevatorStates.isL2Coral)
                .whileTrue(l2Coral());
        L2Coral.and(backwardMode, actionPrepState, ElevatorStates.isL2Coral)
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getL2Coral())); // TODO: change back to command
        // when reversed fixed
        L2Coral.and(backwardMode.not(), actionState).whileTrue(score2());
        L2Coral.and(backwardMode, actionState)
                .whileTrue(
                        elbow.moveToDegrees(
                                () ->
                                        -config.getL2Coral()
                                                + 15)); // TODO: change back to command when
        // reversed fixed

        L3Coral.and(backwardMode.not(), actionPrepState, ElevatorStates.isL3Coral)
                .whileTrue(l3Coral());
        L3Coral.and(backwardMode, actionPrepState, ElevatorStates.isL3Coral)
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getL3Coral())); // TODO: change back to command

        L3Coral.and(backwardMode.not(), actionState).whileTrue(score3());
        L3Coral.and(backwardMode, actionState)
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getL3Coral() + 15)); // TODO: change back to command

        L4Coral.and(backwardMode.not(), actionPrepState, ElevatorStates.isL4Coral)
                .whileTrue(l4Coral());
        L4Coral.and(backwardMode, actionPrepState, ElevatorStates.isL4Coral)
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getL4Coral())); // TODO: change back to command

        L4Coral.and(backwardMode.not(), actionState).whileTrue(score4());
        L4Coral.and(backwardMode, actionState)
                .whileTrue(
                        elbow.moveToDegrees(
                                () -> -config.getL4Coral() + 15)); // TODO: change back to command

        L2Algae.and(actionState.or(actionPrepState))
                .whileTrue(elbow.moveToDegrees(config::getL2Algae));
        L3Algae.and(actionState.or(actionPrepState))
                .whileTrue(elbow.moveToDegrees(config::getL3Algae));

        netAlgae.whileTrue(elbow.moveToDegrees(config::getBarge));
        // climbPrep
        //
        // .whileTrue(elbow.moveToDegrees(config::getClimbPrep).withName("Elbow.startClimb"));

        // barge.and(backwardMode.not()).whileTrue(log(barge()));
        // barge.and(backwardMode).whileTrue(log(reverse(barge())));
        // homeAll.whileTrue(log(home()));

        // algaeHandoff.whileTrue(log(handOffAlgae()));
        // coralHandoff.whileTrue(log(handOffAlgae()));

    }

    private static DoubleSupplier getPosition() {
        return () -> (elbow.getPositionDegrees() + 90);
    }

    private static Command score() {
        return elbow.moveToRelativePosition(() -> -15).withName("Elbow.score");
    }

    private static Command score2() {
        double newPos = config.getL2Coral() + 15;
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score2");
        // return elbow.moveToRelativePosition(() -> -15).withName("Elbow.score2");
    }

    private static Command score3() {
        double newPos = 15 + config.getL3Coral();
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score3");
    }

    private static Command score4() {
        double newPos = 40 + config.getL4Coral();
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score4");
    }

    private static Command home() {
        return elbow.moveToDegrees(config::getHome).withName("Elbow.home");
    }

    private static Command handOffAlgae() {
        return elbow.moveToDegrees(config::getHandAlgae).withName("Elbow.handOffAlgae");
    }

    /* Scoring positions */
    private static Command l2Algae() {
        return elbow.moveToDegreesAndCheckReversed(config::getL2Algae).withName("Elbow.l2Algae");
    }

    private static Command l3Algae() {
        return elbow.moveToDegreesAndCheckReversed(config::getL3Algae).withName("Elbow.l3Algae");
    }

    private static Command barge() {
        return elbow.moveToDegreesAndCheckReversed(config::getBarge).withName("Elbow.barge");
    }

    private static Command l1Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL1Coral).withName("Twist.L1Coral");
    }

    private static Command l2Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL2Coral).withName("Elbow.l2Coral");
    }

    private static Command l3Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL3Coral).withName("Elbow.l3Coral");
    }

    private static Command l4Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL4Coral).withName("Elbow.l4Coral");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command floorIntake() {
        return elbow.moveToDegrees(config::getFloorIntake).withName("Elbow.FloorIntake");
    }

    public static Command stationIntake() {
        return elbow.moveToDegreesAndCheckReversed(config::getStationIntake)
                .withName("Elbow.StationIntake");
    }

    public static Command stationExtendedIntake() {
        return elbow.moveToDegreesAndCheckReversed(config::getStationExtendedIntake)
                .withName("Shoulder.stationExtendedIntake");
    }

    public static Command coastMode() {
        return elbow.coastMode().withName("Elbow.CoastMode");
    }

    public static Command stopMotor() {
        return elbow.runStop().withName("Elbow.stop");
    }

    public static Command ensureBrakeMode() {
        return elbow.ensureBrakeMode().withName("Elbow.BrakeMode");
    }

    // Tune value command
    public static Command tuneElbow() {
        // return elbow.moveToDegrees(new TuneValue("Tune Elbow", 0).getSupplier())
        //         .withName("Elbow.Tune");
        return elbow.moveToDegrees(config::getTuneElbow);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    // Negate position command
    protected static Command reverse(Command cmd) {
        // return cmd.deadlineFor(
        //         Commands.startEnd(() -> config.setReversed(true), () ->
        // config.setReversed(false)));
        return Commands.runOnce(() -> config.setReversed(true))
                .andThen(cmd)
                .andThen(() -> config.setReversed(false));
    }
}
