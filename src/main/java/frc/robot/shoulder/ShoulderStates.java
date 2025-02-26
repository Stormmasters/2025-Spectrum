package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
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
        coastMode.onTrue(log(coastMode()).ignoringDisable(true));
        coastMode.onFalse(log(ensureBrakeMode()));
        photonCoastMode.onTrue(photonShoulder.coastMode().ignoringDisable(true));
        photonCoastMode.onFalse(photonShoulder.ensureBrakeMode());
        // stationIntaking.and(backwardMode.not()).whileTrue(log(stationIntake()));
        // stationIntaking.and(backwardMode).whileTrue(log(reverse(stationIntake())));
        // stationIntaking.whileTrue(shoulder.moveToDegrees((config::getStationIntake)));
        // stationExtendedIntake.and(backwardMode.not()).whileTrue(log(stationExtendedIntake()));
        // stationExtendedIntake.and(backwardMode).whileTrue(log(reverse(stationExtendedIntake())));

        // L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        // L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        // L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        // L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));
        // barge.and(backwardMode.not()).whileTrue(log(barge()));
        // barge.and(backwardMode).whileTrue(log(reverse(barge())));

        // L1Coral.and(backwardMode.not()).whileTrue(log(l1Coral()));
        // L1Coral.and(backwardMode).whileTrue(log(reverse(l1Coral())));
        // L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        // L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        // L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        // L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));
        // L2Coral.and(backwardMode.not(), actionPrepState, ElevatorStates.isL2Coral)
        //         .whileTrue(l2Coral());
        // L2Coral.and(backwardMode, actionPrepState, ElevatorStates.isL2Coral)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () -> -config.getL2Coral())); // TODO: change back to command

        // L2Coral.and(backwardMode.not(), scoreState).whileTrue(score2());
        // L2Coral.and(backwardMode, scoreState)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () ->
        //                                 -config.getL2Coral()
        //                                         + 15)); // TODO: change back to command when

        // L3Coral.and(backwardMode.not(), actionPrepState, ElevatorStates.isL3Coral)
        //         .whileTrue(l3Coral());
        // L3Coral.and(backwardMode, actionPrepState, ElevatorStates.isL3Coral)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () -> -config.getL3Coral())); // TODO: change back to command

        // L3Coral.and(backwardMode.not(), scoreState).whileTrue(score3());
        // L3Coral.and(backwardMode, scoreState)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () ->
        //                                 -config.getL3Coral()
        //                                         + 15)); // TODO: change back to command when

        // L4Coral.and(backwardMode.not(), actionPrepState).whileTrue(l4Coral());
        // L4Coral.and(backwardMode, actionPrepState)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () -> -config.getL4Coral())); // TODO: change back to command

        // L4Coral.and(backwardMode.not(), scoreState).whileTrue(score4());
        // L4Coral.and(backwardMode, scoreState, ElbowStates.atTarget)
        //         .whileTrue(
        //                 shoulder.moveToDegrees(
        //                         () ->
        //                                 -config.getL4Coral()
        //                                         + 15)); // TODO: change back to command when

        // L3Coral.and(backwardMode.not()).whileTrue(log(l3Coral()));
        // L3Coral.and(backwardMode).whileTrue(log(reverse(l3Coral())));
        // L4Coral.and(backwardMode.not()).whileTrue(log(l4Coral()));
        // L4Coral.and(backwardMode).whileTrue(log(reverse(l4Coral())));

        // algaeHandoff.whileTrue(log(handOffAlgae()));
        // coralHandoff.whileTrue(log(handOffAlgae()));

        // homeAll.whileTrue(home());

        // TODO: for testing
        // Robot.getPilot().testTune_tA.whileTrue(shoulder.moveToDegrees(config::getStationIntake));
        // // Robot.getPilot().testTune_tB.whileTrue(shoulder.moveToDegrees(config::getL3Algae));
        // Robot.getPilot().testTune_tX.whileTrue(shoulder.moveToDegrees(config::getHome));
        // // Robot.getPilot()
        // //         .testTune_tY
        // //         .whileTrue(shoulder.moveToDegrees(() -> (config.getL4Coral() - 6)));
        // Robot.getPilot().reZero_start.onTrue(shoulder.resetToInitialPos());
        // // Robot.getPilot()
        // //         .testTriggersTrigger
        // //         .whileTrue(
        // //                 runShoulder(() -> Robot.getPilot().getTestTriggersAxis())
        // //                         .withName("test Shoulder"));
        // Robot.getOperator().test_tA.whileTrue(shoulder.moveToDegrees(config::getL1Coral));
        // Robot.getOperator().test_tB.whileTrue(shoulder.moveToDegrees(config::getL2Coral));
        // // Robot.getOperator().test_tX.whileTrue(shoulder.moveToDegrees(config::getL3Coral));
        // Robot.getOperator()
        //         .test_tX
        //         .and(backwardMode.not())
        //         .whileTrue(shoulder.moveToDegreesAndCheckReversed(config::getL3Coral));
        // Robot.getOperator()
        //         .test_tX
        //         .and(backwardMode)
        //         //
        // .whileTrue(reverse(shoulder.moveToDegreesAndCheckReversed(config::getL3Coral)));
        //         .whileTrue(shoulder.moveToDegreesAndCheckReversed(() -> -config.getL3Coral()));
        // Robot.getOperator().test_tY.whileTrue(shoulder.moveToDegrees(config::getL4Coral));
        // Robot.getOperator().test_A.whileTrue(shoulder.moveToDegrees(config::getL2Algae));
        // Robot.getOperator().test_B.whileTrue(shoulder.moveToDegrees(config::getL3Algae));
        // Robot.getOperator().test_X.whileTrue(shoulder.moveToDegrees(config::getBarge));
        // homeAll.whileTrue(home());

        // PHOTON states
        // Robot.getPhotonPilot()
        //         .testTriggersTrigger
        //         .whileTrue(
        //                 runPhotonShoulder(() -> Robot.getPhotonPilot().getTestTriggersAxis())
        //                         .withName("test PhotonShoulder"));
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
