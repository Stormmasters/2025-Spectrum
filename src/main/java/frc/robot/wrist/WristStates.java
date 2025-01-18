package frc.robot.wrist;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Robot;
import frc.robot.wrist.Wrist.WristConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class WristStates {
    private static Wrist wrist = Robot.getWrist();
    private static WristConfig config = Robot.getConfig().wrist;

    public static void setupDefaultCommand() {
        wrist.setDefaultCommand(
                log(wrist.runHoldWrist().ignoringDisable(true).withName("Wrist.default")));
    }

    public static void setStates() {
        coastMode.onTrue(
                log(coastMode())
                        .alongWith(new PrintCommand("this works again").ignoringDisable(true)));
        coastMode.onFalse(log(ensureBrakeMode()));
        intaking.whileTrue(tuneWrist()); // using intake button to test
        score.whileTrue(home());
        algaeFloor.whileTrue(moveToPercentage(config::getFloorIntake));
        lTwoAlgae.whileTrue(lTwoAlgae());
        lThreeAlgae.whileTrue(lThreeAlgae());
        lTwoCoral.whileTrue(lTwoCoral());
        lThreeCoral.whileTrue(lThreeCoral());
        // home.whileTrue(home());
    }

    public static Command runWrist(DoubleSupplier speed) {
        return wrist.runPercentage(speed).withName("Wrist.runWrist");
    }

    public static Command home() {
        return wrist.moveToPercentage(config::getHome).withName("Wrist.home");
    }

    public static Command climbHome() {
        return wrist.moveToPercentage(config::getClimbHome).withName("Wrist.climbHome");
    }

    public static Command manualFeed() {
        return wrist.moveToPercentage(config::getManualFeed).withName("Wrist.manualFeed");
    }

    public static Command moveToPercentage(DoubleSupplier percent) {
        return wrist.moveToPercentage(percent).withName("Wrist.moveToPercentage");
    }

    /* Scoring positions */

    public static Command lTwoAlgae() {
        return wrist.moveToPercentage(config::getL2Algae).withName("Wrist.lTwoAlgae");
    }

    public static Command lThreeAlgae() {
        return wrist.moveToPercentage(config::getL3Algae).withName("Wrist.lThreeAlgae");
    }

    public static Command lTwoCoral() {
        return wrist.moveToPercentage(config::getL2Coral).withName("Wrist.lTwoCoral");
    }

    public static Command lThreeCoral() {
        return wrist.moveToPercentage(config::getL3Coral).withName("Wrist.lThreeCoral");
    }

    // missing auton Wrist commands, add when auton is added

    public static Command intake() {
        return wrist.moveToPercentage(config::getIntake).withName("Wrist.intake");
    }

    public static Command coastMode() {
        return wrist.coastMode().withName("Wrist.CoastMode");
    }

    public static Command stopMotor() {
        return wrist.runStop().withName("Wrist.stop");
    }

    public static Command ensureBrakeMode() {
        return wrist.ensureBrakeMode().withName("Wrist.BrakeMode");
    }

    // Tune value command
    public static Command tuneWrist() {
        // return wrist.moveToPercentage(new TuneValue("Tune Wrist", 0).getSupplier())
        //         .withName("Wrist.Tune");
        return wrist.moveToPercentage(config::getTuneWrist);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
