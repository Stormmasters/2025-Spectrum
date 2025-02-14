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
        coralIntake.setDefaultCommand(
                log(coralIntake.runVelocityTcFocRpm(() -> -500).withName("coralIntake.default")));
    }

    public static void setStates() {
        stationIntaking.whileTrue(log(coralIntake()));
        stationExtendedIntake.whileTrue(log(coralIntake()));

        scoreState.and(barge.not()).onTrue(log(coralScore()));
        scoreState.and(barge).onTrue(log(barge()));

        processorLollipopScore.whileTrue(log(coralIntake()));

        algaeHandoff.onTrue(log(algaeHandOff()));
        coralHandoff.onTrue(log(coralHandOff()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command coralHandOff() {
        coralIntake.setDefaultCommand(
                coralIntake.runVelocityTcFocRpm(() -> -500).withName("coralIntake.default"));
        return coralIntake
                .runStop()
                .withName("coralIntake.coralHandOffWait")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 95.0)
                .andThen(coralIntake())
                .withName("coralIntake.coralHandOff");
    }

    private static Command algaeHandOff() {
        coralIntake.setDefaultCommand(
                coralIntake.runVelocityTcFocRpm(() -> 500).withName("coralIntake.default"));
        return coralIntake
                .runStop()
                .withName("coralIntake.algaeHandOffWait")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 95.0)
                .andThen(algaeIntake())
                .withName("coralIntake.algaeHandOff");
    }

    private static Command coralScore() {
        return slowEject()
                .withName("coralIntake.score")
                .until(() -> (!coralIntake.hasIntakeCoral()))
                .withName("coralIntake.scoreDone");
    }

    private static Command barge() {
        return coralIntake
                .runVelocityTcFocRpm(config::getBarge)
                .withName("coralIntake.barge")
                .until(() -> (!coralIntake.hasIntakeAlgae()))
                .withName("coralIntake.bargeScoreDone");
    }

    private static Command coralIntake() {
        coralIntake.setDefaultCommand(
                coralIntake.runVelocityTcFocRpm(() -> -500).withName("coralIntake.default"));
        return coralIntake.runVelocityTcFocRpm(config::getIntake).withName("coralIntake.intake");
    }

    private static Command coralEject() {
        return coralIntake.runVelocityTcFocRpm(config::getEject).withName("coralIntake.eject");
    }

    private static Command algaeIntake() {
        coralIntake.setDefaultCommand(
                coralIntake.runVelocityTcFocRpm(() -> 500).withName("coralIntake.default"));
        return coralIntake
                .runVelocityTcFocRpm(config::getEject)
                .withName("coralIntake.algaeIntake");
    }

    private static Command algaeEject() {
        return coralIntake
                .runVelocityTcFocRpm(config::getIntake)
                .withName("coralIntake.algaeEject");
    }

    private static Command slowEject() {
        return coralIntake
                .runVelocityTcFocRpm(config::getSlowEject)
                .withName("coralIntake.slowEject");
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
