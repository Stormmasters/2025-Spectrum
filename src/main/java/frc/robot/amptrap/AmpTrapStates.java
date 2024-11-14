package frc.robot.amptrap;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.amptrap.AmpTrap.AmpTrapConfig;
import frc.robot.pilot.Pilot;
import java.util.function.DoubleSupplier;

public class AmpTrapStates {
    private static AmpTrap ampTrap;
    private static AmpTrapConfig config;
    private static Pilot pilot = Robot.getPilot();

    public static void setupDefaultCommand() {
        ampTrap.setDefaultCommand(ampTrap.runStop());
    }

    public static void bindTriggers() {

    }

    public static Command runVelocity(DoubleSupplier velocityRPM) {
        return ampTrap.runVelocity(velocityRPM).withName("AmpTrap.runVelocity");
    }

    public static Command runFull() {
        return runVelocity(config::getMaxSpeed).withName("AmpTrap.runFull");
    }

    public static Command feed() {
        return ampTrap.runVelocity(config::getFeed).withName("AmpTrap.feed");
    }

    public static Command intake() {
        return ampTrap.runVelocity(config::getIntake).withName("AmpTrap.intake");
    }

    public static Command amp() {
        return ampTrap.runVelocity(config::getAmp).withName("AmpTrap.amp");
    }

    public static Command score() {
        return ampTrap.runVelocity(config::getScore).withName("AmpTrap.score");
    }

    public static Command eject() {
        return ampTrap.runVelocity(config::getEject).withName("AmpTrap.eject");
    }

    public static Command stayCoastMode() {
        return ampTrap.stayCoastMode();
    }
}
