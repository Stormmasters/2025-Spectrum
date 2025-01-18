package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(
                log(elbow.runHoldElbow().ignoringDisable(true).withName("Elbow.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        intaking.whileTrue(tuneElbow());
        algaeFloor.whileTrue(elbow.moveToPercentage(config::getFloorIntake));
        L2Algae.whileTrue(l2Algae());
        L3Algae.whileTrue(l3Algae());
        L2Coral.whileTrue(l2Coral());
        L3Coral.whileTrue(l3Coral());
        L4Coral.whileTrue(l4Coral());
        // home.whileTrue(home());
        // moveElbow.whileFalse(elbow.runHoldElbow());
        // homeElbow.whileFalse(elbow.runHoldElbow());
        // moveElbow.whileTrue(home());
    }

    public static Command runElbow(DoubleSupplier speed) {
        return elbow.runPercentage(speed).withName("Elbow.runElbow");
    }

    public static Command home() {
        return elbow.moveToPercentage(config::getHome).withName("Elbow.home");
    }

    /* Scoring positions */
    public static Command l2Algae() {
        return elbow.moveToPercentage(config::getL2Algae).withName("Elbow.l2Algae");
    }

    public static Command l3Algae() {
        return elbow.moveToPercentage(config::getL3Algae).withName("Elbow.l3Algae");
    }

    public static Command l2Coral() {
        return elbow.moveToPercentage(config::getL2Coral).withName("Elbow.l2Coral");
    }

    public static Command l3Coral() {
        return elbow.moveToPercentage(config::getL3Coral).withName("Elbow.l3Coral");
    }

    public static Command l4Coral() {
        return elbow.moveToPercentage(config::getL4Coral).withName("Elbow.l4Coral");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command intake() {
        return elbow.moveToPercentage(config::getIntake).withName("Elbow.intake");
    }

    public static Command coastMode() {
        return elbow.coastMode().withName("Elbow.CoastMode");
    }

    public static Command stopMotor() {
        return elbow.runStop().withName("Elbow.stop");
    }

    public static Command ensureBrakeMode() {
        return elbow.ensureBrakeMode().withName("Elbow.BrakeMode");
    }

    // Tune value command
    public static Command tuneElbow() {
        // return elbow.moveToPercentage(new TuneValue("Tune Elbow", 0).getSupplier())
        //         .withName("Elbow.Tune");
        return elbow.moveToPercentage(config::getTuneElbow);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
