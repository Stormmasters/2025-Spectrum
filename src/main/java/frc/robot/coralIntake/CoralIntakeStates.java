package frc.robot.coralIntake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.coralIntake.CoralIntake.CoralIntakeConfig;
import frc.spectrumLib.Telemetry;

public class CoralIntakeStates {
    private static CoralIntake coralIntake = Robot.getCoralIntake();
    private static CoralIntakeConfig config = Robot.getConfig().coralIntake;

    public static void setupDefaultCommand() {
        coralIntake.setDefaultCommand(coralIntake.runStop().withName("coralIntake.default"));
    }

    public static void setStates() {
        homeAllStopIntake.onTrue(coralIntake.getDefaultCommand());

        stationIntaking.onTrue(coralIntake.coralIntake());
        intaking.and(algae).onTrue(coralIntake.algaeIntake());

        L1Coral.and(actionState).whileTrue(coralIntake.coralL1Score());
        L2Coral.and(actionState).whileTrue(coralIntake.coralScore());
        L3Coral.and(actionState).whileTrue(coralIntake.coralScore());
        L4Coral.and(actionState).whileTrue(coralIntake.coralScore());

        processorAlgae.and(actionState).whileTrue(coralIntake.algaeScore());
        L2Algae.and(actionState).whileTrue(coralIntake.algaeIntake());
        L3Algae.and(actionState).whileTrue(coralIntake.algaeIntake());
        netAlgae.and(actionState).whileTrue(coralIntake.algaeScore());

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
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
