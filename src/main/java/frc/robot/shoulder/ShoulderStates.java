package frc.robot.shoulder;

import static frc.robot.RobotStates.*;
import static frc.robot.auton.Auton.autonScore;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;
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

        stationIntaking.whileTrue(
                moveToDegrees(config::getStationIntake, "Shoulder.stationIntake"));
        stationExtendedIntaking.whileTrue(
                moveToDegrees(config::getStationExtendedIntake, "Shoulder.stationExtendedIntake"));
        stationIntaking.or(stationExtendedIntaking).onFalse(home());

        Robot.getPilot()
                .photonRemoveL2Algae
                .whileTrue(moveToDegrees(config::getL2Algae, "Shoulder.L2Algae"));
        Robot.getPilot()
                .photonRemoveL3Alage
                .whileTrue(moveToDegrees(config::getL3Algae, "Shoulder.L3Algae"));
        Robot.getPilot()
                .photonRemoveL2Algae
                .or(Robot.getPilot().photonRemoveL3Alage)
                .onFalse(home());

        L1Coral.and(actionState.or(actionPrepState))
                .whileTrue(moveToDegrees(config::getL1Coral, "Shoulder.L1Coral"));
        L2Coral.and(actionPrepState)
                .whileTrue(moveToDegrees(config::getL2Coral, "Shoulder.L2Coral.prescore"));
        L2Coral.and(actionState)
                .whileTrue(moveToDegrees(config::getL2CoralScore, "Shoulder.L2Coral.score"));
        L3Coral.and(actionPrepState)
                .whileTrue(moveToDegrees(config::getL3Coral, "Shoulder.L3Coral.prescore"));
        L3Coral.and(actionState)
                .whileTrue(moveToDegrees(config::getL3CoralScore, "Shoulder.L3Coral.score"));
        L4Coral.and(actionPrepState)
                .whileTrue(moveToDegrees(config::getL4Coral, "Shoulder.L4Coral.prescore"));
        L4Coral.and(actionState)
                .whileTrue(moveToDegrees(config::getL4CoralScore, "Shoulder.L4Coral.score"));

        processorAlgae
                .and(actionPrepState.or(actionState))
                .whileTrue(moveToDegrees(config::getProcessorAlgae, "Shoulder.processorAlgae"));
        L2Algae.and(actionPrepState.or(actionState))
                .whileTrue(moveToDegrees(config::getL2Algae, "Shoulder.L2Algae"));
        L3Algae.and(actionPrepState.or(actionState))
                .whileTrue(moveToDegrees(config::getL3Algae, "Shoulder.L3Algae"));
        netAlgae.and(actionPrepState.or(actionState))
                .whileTrue(moveToDegrees(config::getNetAlgae, "Shoulder.netAlgae"));

        Robot.getPilot().reZero_start.onTrue(shoulder.resetToIntialPos());
        // climbPrep_start
        //         .whileTrue(moveToDegrees(config::getClimbPrep, "Shoulder.startClimb"));

        autonScore.onTrue(
                new WaitCommand(4.0).andThen(moveToDegrees(() -> 180, "Shoulder.homeAuto")));
    }

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command home() {
        return shoulder.moveToDegrees(config::getHome).withName("Shoulder.home");
    }

    public static DoubleSupplier getPosition() {
        return () -> (shoulder.getPositionDegrees() + 90);
    }

    public static Command moveToDegrees(DoubleSupplier position, String name) {
        return shoulder.moveToDegrees(position).withName(name);
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
