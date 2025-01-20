package frc.robot.algaePivot;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.algaePivot.AlgaePivot.AlgaePivotConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class AlgaePivotStates {
    private static AlgaePivot algaePivot = Robot.getAlgaePivot();
    private static AlgaePivotConfig config = Robot.getConfig().algaePivot;

    public static void setupDefaultCommand() {
        algaePivot.setDefaultCommand(
                log(
                        algaePivot
                                .runHoldalgaePivot()
                                .ignoringDisable(true)
                                .withName("algaePivot.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        intaking.whileTrue(tuneElbow());
        // algaeFloor.whileTrue(elbow.moveToPercentage(config::getFloorIntake));
        // L2Algae.whileTrue(l2Algae());
        // L3Algae.whileTrue(l3Algae());
        // L2Coral.whileTrue(l2Coral());
        // L3Coral.whileTrue(l3Coral());
        // L4Coral.whileTrue(l4Coral());
        // // home.whileTrue(home());
        // // moveElbow.whileFalse(elbow.runHoldElbow());
        // // homeElbow.whileFalse(elbow.runHoldElbow());
        // // moveElbow.whileTrue(home());
    }

    public static Command runElbow(DoubleSupplier speed) {
        return algaePivot.runPercentage(speed).withName("Elbow.runElbow");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command intake() {
        return algaePivot.moveToPercentage(config::getIntake).withName("Elbow.intake");
    }

    public static Command coastMode() {
        return algaePivot.coastMode().withName("Elbow.CoastMode");
    }

    public static Command stopMotor() {
        return algaePivot.runStop().withName("Elbow.stop");
    }

    public static Command ensureBrakeMode() {
        return algaePivot.ensureBrakeMode().withName("Elbow.BrakeMode");
    }

    // Tune value command
    public static Command tuneElbow() {
        // return pivot.moveToPercentage(new TuneValue("Tune Elbow", 0).getSupplier())
        //         .withName("Elbow.Tune");
        return algaePivot.moveToPercentage(config::getTuneElbow);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
