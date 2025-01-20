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
        // missing bindTriggers
        // test.whileTrue(log(home()));

        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        // coastOff.onTrue(log(ensureBrakeMode()));
        lThreeAlgae.whileTrue(elbow.moveToPercentage(() -> -74));
        lTwoAlgae.whileTrue(elbow.moveToPercentage(() -> -97));
        algaeFloor.whileTrue(elbow.moveToPercentage(() -> -92));
        lThreeCoral.whileTrue(elbow.moveToPercentage(() -> -49));
        // test.whileTrue(log(runElbow(() -> .3)));
        // moveElbow.whileTrue(runElbow(() -> 0.1));
        // // home.whileTrue(home());
        // homeElbow.whileTrue(runElbow(() -> -0.1));
        // moveElbow.whileFalse(elbow.runHoldElbow());
        // homeElbow.whileFalse(elbow.runHoldElbow());
        // moveElbow.whileTrue(home());
    }

    // missing distance based Elbow commands

    public static Command runElbow(DoubleSupplier speed) {
        return elbow.runPercentage(speed).withName("Elbow.runElbow");
    }

    public static Command home() {
        return elbow.moveToPercentage(config::getHome).withName("Elbow.home");
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
