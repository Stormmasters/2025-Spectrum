package frc.robot.shoulder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

        coastMode.onTrue(
                log(coastMode())
                        .alongWith(new PrintCommand("this works again").ignoringDisable(true)));
        coastMode.onFalse(log(ensureBrakeMode()));
        // coastOn.whileTrue(coastMode().withName("coastOn"));
        // test.whileTrue(moveToPercentage(config::getNinetyDegrees));
        algaeFloor.whileTrue(moveToPercentage(config::getFloorIntake));
        lTwoAlgae.whileTrue(lTwoAlgae());
        lThreeAlgae.whileTrue(lThreeAlgae());
        lTwoCoral.whileTrue(lTwoCoral());
        lThreeCoral.whileTrue(lThreeCoral());
        // home.whileTrue(home());
        // home.whileTrue(runShoulder(() -> -0.1));
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

    public static Command moveToPercentage(DoubleSupplier percent) {
        return shoulder.moveToPercentage(percent).withName("Shoulder.moveToPercentage");
    }

    /* Scoring positions */

    public static Command lTwoAlgae() {
        return shoulder.moveToPercentage(config::getL2Algae).withName("Shoulder.lTwoAlgae");
    }

    public static Command lThreeAlgae() {
        return shoulder.moveToPercentage(config::getL3Algae).withName("Shoulder.lThreeAlgae");
    }

    public static Command lTwoCoral() {
        return shoulder.moveToPercentage(config::getL2Coral).withName("Shoulder.lTwoCoral");
    }

    public static Command lThreeCoral() {
        return shoulder.moveToPercentage(config::getL3Coral).withName("Shoulder.lThreeCoral");
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
