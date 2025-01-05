package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
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
        ampPrep.whileTrue(log(home()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
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

    public static Command switchFeedSpot() {
        return elbow.runOnce(elbow::switchFeedSpot)
                .withName("Elbow.switchFeedSpot")
                .ignoringDisable(true);
    }

    // Tune value command
    public static Command tuneElbow() {
        return elbow.moveToPercentage(new TuneValue("Tune Elbow", 0).getSupplier())
                .withName("Elbow.Tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
