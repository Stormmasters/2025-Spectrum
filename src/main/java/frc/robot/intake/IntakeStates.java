package frc.robot.intake;

import static frc.robot.RobotStates.*;
import static frc.robot.auton.Auton.autonScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class IntakeStates {
    private static Intake coralIntake = Robot.getIntake();
    private static IntakeConfig config = Robot.getConfig().intake;

    public static void setupDefaultCommand() {
        coralIntake.setDefaultCommand(
                coralIntake.defaultHoldOrStop().ignoringDisable(true).withName("Intake.default"));
    }

    public static void setStates() {
        intakeRunning.onFalse(coralIntake.getDefaultCommand());
        Robot.getPilot()
                .home_select
                .or(Robot.getOperator().home_select)
                .onTrue(coralIntake.getDefaultCommand());

        Trigger photonAlgaeRemoval =
                (Robot.getPilot().photonRemoveL2Algae.or(Robot.getPilot().photonRemoveL3Algae))
                        .and(photon);
        stationIntaking
                .or(photonAlgaeRemoval, stationExtendedIntaking)
                .onFalse(coralIntake.getDefaultCommand());

        netAlgae.and(actionState)
                .whileTrue(
                        runVoltageCurrentLimits(
                                config::getAlgaeScoreVoltage,
                                config::getAlgaeScoreSupplyCurrent,
                                config::getAlgaeScoreTorqueCurrent));

        stationIntaking
                .or(photonAlgaeRemoval, stationExtendedIntaking)
                // .whileTrue(runVoltageCurrentLimits(
                //         config::getCoralIntakeVoltage,
                //         config::getCoralIntakeSupplyCurrent,
                //         config::getCoralIntakeTorqueCurrent));
                .whileTrue(runGroundIntake());

        groundCoral.whileTrue(runGroundIntake());

        algae.and(photon.not())
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

        autonScore
                .and(photon)
                .onTrue(
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

    private static Command runGroundIntake() {
        return coralIntake.runTCcurrentLimits(
                config::getCoralGroundTorqueCurrent, config::getCoralGroundSupplyCurrent);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
