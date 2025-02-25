package frc.robot.coralIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.coralIntake.CoralIntake.CoralIntakeConfig;
import frc.robot.elbow.ElbowStates;
import frc.spectrumLib.Telemetry;

public class CoralIntakeStates {
    private static CoralIntake coralIntake = Robot.getCoralIntake();
    private static CoralIntakeConfig config = Robot.getConfig().coralIntake;

    public static void setupDefaultCommand() {
        coralIntake.setDefaultCommand(coralIntake.runStop().withName("coralIntake.default"));
    }

    public static void setStates() {
        // TODO: Fix this after testing
        stationIntaking.onTrue(coralIntake.coralIntake()); // log(coralIntake()));
        // stationExtendedIntake.whileTrue(coralEject()); // log(coralIntake()));

        L2Coral.and(scoreState.not()).onTrue(coralIntake.coralIntake());
        L2Coral.and(scoreState).whileTrue(coralIntake.coralScore());
        L3Coral.and(scoreState.not()).onTrue(coralIntake.coralIntake());
        L3Coral.and(scoreState).whileTrue(coralIntake.coralScore());
        L4Coral.and(scoreState.not()).onTrue(coralIntake.coralIntake());
        L4Coral.and(scoreState).whileTrue(coralIntake.coralScore());

        homeAllStopIntake.onTrue(coralIntake.runStop());

        // scoreState.and(barge.not()).onTrue(log(coralScore()));
        // scoreState.and(barge).onTrue(log(barge()));

        // processorLollipopScore.whileTrue(log(coralIntake()));

        // passiveCoral.whileTrue(coralIntake.coralIntake()); // TODO: delete?
        // passiveAlgae.whileTrue(coralIntake.algaeIntake());
        // hasAlgae.whileTrue(log(algaeIntake()));

        // algaeHandoff.onTrue(log(algaeHandOff()));
        // coralHandoff.onTrue(log(coralHandOff()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        Robot.getPilot().testTune_tA.whileTrue(coralIntake.coralIntake());
        // Robot.getPilot().testTune_tB.whileTrue(coralIntake.algaeIntake());
        Robot.getPilot().testTune_tX.whileTrue(coralIntake.coralIntake());
        // Robot.getPilot().testTune_tY.whileTrue(coralIntake.coralScore());
        // Robot.getPilot().tuneShoulder_tB.whileTrue(coralIntake.coralScore());
        // Robot.getOperator().test_tA.whileTrue(coralIntake.coralIntake());
        // Robot.getPilot().testTune_RB.whileTrue(coralIntake.algaeScore());
        Robot.getOperator().test_X.whileTrue(coralIntake.algaeIntake());
        Robot.getOperator().test_A.whileTrue(coralIntake.algaeIntake());
        Robot.getOperator().test_B.whileTrue(coralIntake.algaeIntake());
    }

    private static Command algaeHandOff() {
        return coralIntake
                .runStop()
                .withName("coralIntake.algaeHandOffWait")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 95.0)
                .andThen(algaeIntake())
                .withName("coralIntake.algaeHandOff");
    }

    private static Command barge() {
        return coralIntake
                .runVelocityTcFocRpm(config::getBarge)
                .withName("coralIntake.barge")
                .until(() -> (!coralIntake.hasIntakeAlgae()))
                .withName("coralIntake.bargeScoreDone");
    }

    private static Command algaeIntake() {
        return coralIntake
                .runVelocityTcFocRpm(config::getEject)
                .withName("coralIntake.algaeIntake");
    }

    private static Command algaeEject() {
        return coralIntake
                .runVelocityTcFocRpm(config::getIntake)
                .withName("coralIntake.algaeEject");
    }

    private static Command coastMode() {
        return coralIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return coralIntake.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
