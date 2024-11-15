package frc.robot.feeder;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.feeder.Feeder.FeederConfig;
import frc.robot.pilot.Pilot;

public class FeederStates {
    private static Feeder feeder;
    private static FeederConfig config;
    private static Pilot pilot = Robot.getPilot();

    // TODO: implement feeder states
    /* Check Elevator States */

    public static void setupDefaultCommand() {
        feeder.setDefaultCommand(feeder.runStop().ignoringDisable(true).withName("Feeder.default"));
    }

    public static void bindTriggers() {}

    public static Command runFull() {
        return feeder.runVelocity(config::getMaxSpeed).withName("Feeder.runFull");
    }

    public static Command addFeedRevolutions() {
        return feeder.runAddPosition(config::getAddedFeedRotations)
                .withName("Feeder.addFeedRevolutions");
    }

    public static Command eject() {
        return feeder.runVelocity(config::getEject).withName("Feeder.eject");
    }

    public static Command score() {
        return feeder.runVelocity(config::getScore).withName("Feeder.score");
    }

    public static Command autoFeed() {
        return feeder.runVelocity(config::getAutoFeed).withName("Feeder.autoFeed");
    }

    public static Command ejectFromIntake() {
        return feeder.runVelocity(config::getEjectFromIntake).withName("Feeder.ejectFromIntake");
    }

    public static Command slowFeed() {
        return feeder.runVelocity(config::getSlowFeed).withName("Feeder.slowFeed");
    }

    public static Command slowEject() {
        return feeder.runVelocity(config::getSlowEject).withName("Feeder.slowEject");
    }

    public static Command intake() {
        return feeder.runVelocity(config::getIntake).withName("Feeder.intake");
    }

    public static Command feedToAmp() {
        return feeder.runVelocity(config::getFeedToAmp).withName("Feeder.feedToAmp");
    }

    public static Command launchEject() {
        return feeder.runVelocity(config::getLaunchEject).withName("Feeder.launchEject");
    }

    public static Command manualSource() {
        return feeder.runVelocity(config::getManualSource).withName("Feeder.manualSource");
    }
}
