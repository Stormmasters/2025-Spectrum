package frc.robot.fakeArm;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.fakeArm.FakeArm.FakeArmConfig;
import java.util.function.DoubleSupplier;

public class FakeArmStates {
    private static FakeArm fakeArm = Robot.getFakeArm();
    private static FakeArmConfig config = Robot.getConfig().fakeArm;

    public static void setupDefaultCommand() {
        fakeArm.setDefaultCommand(
                fakeArm.runHoldFakeArm().ignoringDisable(true).withName("FakeArm.default"));
    }

    public static void setStates() {
        // missing bindTriggers
        intaking.whileTrue(subwoofer());
        ampPrep.whileTrue(home());

        coastMode.whileTrue(coastMode());
    }

    // missing distance based fakeArm commands

    public static Command runFakeArm(DoubleSupplier speed) {
        return fakeArm.runPercentage(speed).withName("FakeArm.runFakeArm");
    }

    public static Command home() {
        return fakeArm.moveToPosePercentage(config::getHome).withName("FakeArm.home");
    }

    public static Command climbHome() {
        return fakeArm.moveToPosePercentage(config::getClimbHome).withName("FakeArm.climbHome");
    }

    public static Command manualFeed() {
        return fakeArm.moveToPosePercentage(config::getManualFeed).withName("FakeArm.manualFeed");
    }

    /* Scoring positions */

    public static Command subwoofer() {
        return fakeArm.moveToPosePercentage(config::getSubwoofer).withName("FakeArm.subwoofer");
    }

    public static Command podium() {
        return fakeArm.moveToPosePercentage(config::getPodium).withName("FakeArm.podium");
    }

    public static Command ampWing() {
        return fakeArm.moveToPosePercentage(config::getAmpWing).withName("FakeArm.ampWing");
    }

    public static Command fromAmp() {
        return fakeArm.moveToPosePercentage(config::getFromAmp).withName("FakeArm.fromAmp");
    }

    public static Command intoAmp() {
        return fakeArm.moveToPosePercentage(config::getIntoAmp).withName("FakeArm.intoAmp");
    }

    // missing auton fakeArm commands, add when auton is added

    public static Command intake() {
        return fakeArm.moveToPosePercentage(config::getIntake).withName("FakeArm.intake");
    }

    public static Command coastMode() {
        return fakeArm.coastMode().withName("FakeArm.CoastMode");
    }

    public static Command stopMotor() {
        return fakeArm.runStop().withName("FakeArm.stop");
    }
}
