package frc.robot.coralIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.coralIntake.CoralIntake.CoralIntakeConfig;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Telemetry;

public class CoralIntakeStates {
    private static CoralIntake coralIntake = Robot.getCoralIntake();
    private static CoralIntakeConfig config = Robot.getConfig().coralIntake;

    public static void setupDefaultCommand() {
        coralIntake.setDefaultCommand(
                log(coralIntake.runVelocityTcFocRpm(() -> -500).withName("intake.default")));
    }

    public static void setStates() {
        stationIntaking.whileTrue(log(intake()));
        scoreState.whileTrue(log(score()));

        algaeHandoff.whileTrue(log(handOff()));
        coralHandoff.whileTrue(log(handOff()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command handOff() {
        return coralIntake
                .runStop()
                .withName("coralIntake.handOffWait")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 95.0)
                .andThen(intake())
                .withName("coralIntake.handOff");
    }

    private static Command score() {
        double originalPosition = ElevatorStates.getPosition().getAsDouble();
        return coralIntake
                .runStop()
                .withName("coralIntake.scoreWait")
                .until(() -> ((ElevatorStates.getPosition().getAsDouble() - originalPosition) > 1))
                .andThen(eject())
                .withName("coralIntake.score")
                .until(() -> (!coralIntake.hasIntakeCoral()))
                .withName("coralIntake.scoreDone");
    }

    private static Command intake() {
        return coralIntake.runVelocityTcFocRpm(config::getIntake).withName("coralIntake.intake");
    }

    private static Command eject() {
        return coralIntake.runVelocityTcFocRpm(config::getEject).withName("coralIntake.eject");
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
