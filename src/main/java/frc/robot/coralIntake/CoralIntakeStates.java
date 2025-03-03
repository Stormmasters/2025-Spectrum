package frc.robot.coralIntake;

import static frc.robot.RobotStates.*;
import static frc.robot.auton.Auton.autonScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.coralIntake.CoralIntake.CoralIntakeConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class CoralIntakeStates {
    private static CoralIntake coralIntake = Robot.getCoralIntake();
    private static CoralIntakeConfig config = Robot.getConfig().coralIntake;

    public static void setupDefaultCommand() {
        coralIntake.setDefaultCommand(
                coralIntake.runStop().ignoringDisable(true).withName("coralIntake.default"));
    }

    public static void setStates() {
        homeAllStopIntake.onTrue(coralIntake.getDefaultCommand());
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(coralIntake.getDefaultCommand());

        Trigger photonAlageRemoval =
                Robot.getPilot().photonRemoveL2Algae.or(Robot.getPilot().photonRemoveL3Alage);
        photonAlageRemoval.or(stationExtendedIntaking).onFalse(coralIntake.getDefaultCommand());

        netAlgae.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getAlgaeScoreVoltage,
                                config::getAlgaeScoreSupplyCurrent,
                                config::getAlgaeScoreTorqueCurrent));

        stationIntaking
                .or(photonAlageRemoval, stationExtendedIntaking)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getCoralIntakeVoltage,
                                config::getCoralIntakeSupplyCurrent,
                                config::getCoralIntakeTorqueCurrent));

        intaking.and(algae)
                .onTrue(
                        runVoltageCurrentLimits(
                                config::getAlgaeIntakeVoltage,
                                config::getAlgaeIntakeSupplyCurrent,
                                config::getAlgaeIntakeTorqueCurrent));

        L1Coral.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getCoralL1ScoreVoltage,
                                config::getCoralL1ScoreSupplyCurrent,
                                config::getCoralL1ScoreTorqueCurrent));
        branch.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getCoralScoreVoltage,
                                config::getCoralScoreSupplyCurrent,
                                config::getCoralScoreTorqueCurrent));

        stagedAlgae
                .and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getAlgaeIntakeVoltage,
                                config::getAlgaeIntakeSupplyCurrent,
                                config::getAlgaeIntakeTorqueCurrent));

        autonScore.onTrue(
                new WaitCommand(2.0)
                        .andThen(
                                runVoltageCurrentLimits(
                                                config::getCoralScoreVoltage,
                                                config::getCoralScoreSupplyCurrent,
                                                config::getCoralScoreTorqueCurrent)
                                        .repeatedly()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command coastMode() {
        return coralIntake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return coralIntake.ensureBrakeMode();
    }

    private static Command runVoltageCurrentLimits(
            DoubleSupplier voltage, DoubleSupplier supplyCurrent, DoubleSupplier torqueCurrent) {
        return coralIntake.runVoltageCurrentLimits(voltage, supplyCurrent, torqueCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
