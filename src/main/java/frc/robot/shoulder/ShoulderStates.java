package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.shoulder.Shoulder.ShoulderConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ShoulderStates {
    private static Shoulder shoulder = Robot.getShoulder();
    private static ShoulderConfig config = Robot.getConfig().shoulder;

    public static void setupDefaultCommand() {
        shoulder.setDefaultCommand(
                log(shoulder.runHoldShoulder().ignoringDisable(true).withName("Shoulder.default")));
    }

    public static void setStates() {
        // missing bindTriggers
        intaking.whileTrue(log(subwoofer()));
        ampPrep.whileTrue(log(home()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    // missing distance based Shoulder commands

    public static Command runShoulder(DoubleSupplier speed) {
        return shoulder.runPercentage(speed).withName("Shoulder.runShoulder");
    }

    public static Command home() {
        return shoulder.moveToPercentage(config::getHome).withName("Shoulder.home");
    }

    public static Command climbHome() {
        return shoulder.moveToPercentage(config::getClimbHome).withName("Shoulder.climbHome");
    }

    public static Command manualFeed() {
        return shoulder.moveToPercentage(config::getManualFeed).withName("Shoulder.manualFeed");
    }

    /* Scoring positions */

    public static Command subwoofer() {
        return shoulder.moveToPercentage(config::getSubwoofer).withName("Shoulder.subwoofer");
    }

    public static Command podium() {
        return shoulder.moveToPercentage(config::getPodium).withName("Shoulder.podium");
    }

    public static Command ampWing() {
        return shoulder.moveToPercentage(config::getAmpWing).withName("Shoulder.ampWing");
    }

    public static Command fromAmp() {
        return shoulder.moveToPercentage(config::getFromAmp).withName("Shoulder.fromAmp");
    }

    public static Command intoAmp() {
        return shoulder.moveToPercentage(config::getIntoAmp).withName("Shoulder.intoAmp");
    }

    // missing auton Shoulder commands, add when auton is added

    public static Command intake() {
        return shoulder.moveToPercentage(config::getIntake).withName("Shoulder.intake");
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

    public static Command switchFeedSpot() {
        return shoulder.runOnce(shoulder::switchFeedSpot)
                .withName("Shoulder.switchFeedSpot")
                .ignoringDisable(true);
    }

    // Tune value command
    public static Command tuneShoulder() {
        return shoulder.moveToPercentage(new TuneValue("Tune Shoulder", 0).getSupplier())
                .withName("Shoulder.Tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
