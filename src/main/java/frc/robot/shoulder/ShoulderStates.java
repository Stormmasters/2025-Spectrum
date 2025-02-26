package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.shoulder.PhotonShoulder.PhotonShoulderConfig;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static PhotonShoulder photonShoulder = Robot.getPhotonShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;
    private static PhotonShoulderConfig photonConfig = Robot.getConfig().photonShoulder;
    public static final Trigger isHome =
            shoulder.atDegrees(() -> (config.getHome() + config.getOffset()), config::getTolerance);

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().withName("Shoulder.HoldDefault")));
        // shoulder.runStop());
    }

    public static void setStates() {
        homeAll.whileTrue(home());
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));
        photonCoastMode.onTrue(photonShoulder.coastMode().ignoringDisable(true));
        photonCoastMode.onFalse(photonShoulder.ensureBrakeMode());
       
        L3Coral.and(backwardMode.not(), scoring).whileTrue(score3());
        L3Coral.and(backwardMode, scoring)
                .whileTrue(
                        shoulder.moveToDegrees(
                                () ->
                                        -config.getL3Coral()
                                                + 15)); // TODO: change back to command when

        // L4Coral.and(backwardMode.not(), actionPrepState).whileTrue(l4Coral());
        // L4Coral.and(backwardMode, actionPrepState)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () -> -config.getL4Coral())); // TODO: change back to command

        L4Coral.and(backwardMode.not(), scoring).whileTrue(score4());
        L4Coral.and(backwardMode, scoring, ElbowStates.atTarget)
                .whileTrue(
                        shoulder.moveToDegrees(
                                () ->
                                        -config.getL4Coral()
                                                + 15)); // TODO: change back to command when

        Robot.getPilot().reZero_start.onTrue(shoulder.resetToIntialPos());
       
        Robot.getPhotonPilot().testTune_tX.whileTrue(photonShoulder.moveToDegrees(() -> 90));
        Robot.getPhotonPilot().testTune_tB.whileTrue(photonShoulder.moveToDegrees(() -> 0));
        Robot.getPhotonPilot()
                .testTune_tA
                .whileTrue(photonShoulder.moveToDegrees(photonConfig::getStationIntake));
    }

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command runPhotonShoulder(DoubleSupplier speed) {
        return photonShoulder.runPercentage(speed).withName("PhotonShoulder.runPhotonShoulder");
    }

    public static Command home() {
        return shoulder.moveToDegrees(config::getHome).withName("Shoulder.home");
    }

    public static DoubleSupplier getPosition() {
        return () -> (shoulder.getPositionDegrees() + 90);
    }

    public static Command climbHome() {
        return shoulder.moveToDegrees(config::getClimbHome).withName("Shoulder.climbHome");
    }

    public static Command handOffAlgae() {
        return shoulder.moveToDegrees(config::getHandAlgae).withName("Elbow.handOffAlgae");
    }

    /* Scoring positions */

    public static Command score2() {
        double newPos = config.getL2Coral() - 15;
        return shoulder.moveToDegreesAndCheckReversed(() -> newPos).withName("Shoulder.score2");
        // return elbow.moveToRelativePosition(() -> -15).withName("Elbow.score2");
    }

    public static Command score3() {
        double newPos = -15 + config.getL3Coral();
        return shoulder.moveToDegreesAndCheckReversed(() -> newPos).withName("Shoulder.score3");
    }

    public static Command score4() {
        double newPos = 50 + config.getL4Coral();
        return new WaitCommand(0.2)
                .andThen(shoulder.moveToDegreesAndCheckReversed(() -> newPos))
                .withName("Shoulder.score3");
    }

    public static Command l2Algae() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL2Algae)
                .withName("Shoulder.l2Algae");
    }

    public static Command l3Algae() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL3Algae)
                .withName("Shoulder.l3Algae");
    }

    public static Command l1Coral() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL1Coral).withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL2Coral)
                .withName("Shoulder.l2Coral");
    }

    public static Command l3Coral() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL3Coral)
                .withName("Shoulder.l3Coral");
    }

    public static Command l4Coral() {
        return shoulder.moveToDegreesAndCheckReversed(config::getL4Coral)
                .withName("Shoulder.l4Coral");
    }

    public static Command barge() {
        return shoulder.moveToDegreesAndCheckReversed(config::getBarge).withName("Shoulder.barge");
    }

    public static Command floorIntake() {
        return shoulder.moveToDegrees(config::getClawGroundAlgaeIntake)
                .withName("Shoulder.floorIntake");
    }

    public static Command stationIntake() {
        return shoulder.moveToDegreesAndCheckReversed(config::getStationIntake)
                .withName("Shoulder.coralIntake");
    }

    public static Command stationExtendedIntake() {
        return shoulder.moveToDegreesAndCheckReversed(config::getStationExtendedIntake)
                .withName("Shoulder.coralExtendedIntake");
    }

    public static Command coastMode() {
        return shoulder.coastMode().withName("Shoulder.CoastMode");
    }

    public static Command stopMotor() {
        return shoulder.runStop().withName("Shoulder.stop");
    }

    public static Command ensureBrakeMode() {
        return shoulder.ensureBrakeMode().withName("Shoulder.BrakeMode");
    }

    // Tune value command
    public static Command tuneShoulder() {
        // return shoulder.moveToDegrees(new TuneValue("Tune Shoulder", 0).getSupplier())
        //         .withName("Shoulder.Tune");
        return shoulder.moveToDegrees(config::getTuneShoulder);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    // Check robot side command
    protected static Command reverse(Command cmd) {
        // return cmd.deadlineFor(
        //         Commands.startEnd(() -> config.setReversed(true), () ->
        // config.setReversed(false)));
        return Commands.runOnce(() -> config.setReversed(true))
                .andThen(cmd)
                .andThen(() -> config.setReversed(false));
    }
}
