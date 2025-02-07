package frc.robot.algaePivot;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.algaePivot.AlgaePivot.AlgaePivotConfig;
import frc.robot.elbow.ElbowStates;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class AlgaePivotStates {
    private static AlgaePivot algaePivot = Robot.getAlgaePivot();
    private static AlgaePivotConfig config = Robot.getConfig().algaePivot;

    public static void setupDefaultCommand() {
        algaePivot.setDefaultCommand(
                log(
                        algaePivot
                                .runHoldAlgaePivot()
                                .ignoringDisable(true)
                                .withName("algaePivot.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        climbPrep.whileTrue(log(climbPrep()));
        homeAll.onTrue(log(home()));
        processorScore.whileTrue(log(processorScore()));
    }

    public static Command runAlgaePivot(DoubleSupplier speed) {
        return algaePivot.runPercentage(speed).withName("AlgaePivot.runAlgaePivot");
    }

    // missing auton AlgaePivot commands, add when auton is added

    public static Command intake() {
        return algaePivot.moveToPercentage(config::getIntake).withName("AlgaePivot.intake");
    }

    public static Command home() {
        return algaePivot.moveToPercentage(config::getHome).withName("AlgaePivot.home");
    }

    public static Command climbPrep() {
        return algaePivot
                .moveToPercentage(config::getPrepClimber)
                .withName("AlgaePivot.prepClimber");
    }

    public static Command algaeFloorIntake() {
        return algaePivot
                .runHoldAlgaePivot()
                .withName("AlgaePivot.waitingForElbow")
                .until(() -> (ElbowStates.getPosition().getAsDouble() < 10.0))
                .andThen(
                        algaePivot
                                .moveToPercentage(config::getFloorIntake)
                                .withName("AlgaePivot.floorIntake"));
    }

    public static Command coralIntake() {
        return algaePivot
                .moveToPercentage(config::getCoralIntake)
                .withName("AlgaePivot.coralIntake");
    }

    public static Command processorScore() {
        return algaePivot
                .moveToPercentage(config::getProcessorScore)
                .withName("AlgaePivot.processorScore");
    }

    public static DoubleSupplier getPosition() {
        return (() -> algaePivot.getPositionPercentage());
    }

    public static Command coastMode() {
        return algaePivot.coastMode().withName("AlgaePivot.CoastMode");
    }

    public static Command stopMotor() {
        return algaePivot.runStop().withName("AlgaePivot.stop");
    }

    public static Command ensureBrakeMode() {
        return algaePivot.ensureBrakeMode().withName("AlgaePivot.BrakeMode");
    }

    // Tune value command
    public static Command tuneAlgaePivot() {
        // return pivot.moveToPercentage(new TuneValue("Tune AlgaePivot", 0).getSupplier())
        //         .withName("AlgaePivot.Tune");
        return algaePivot.moveToPercentage(config::getTuneAlgaePivot);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
