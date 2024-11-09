package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/** This class should have any command calls that directly call the Pilot */
public class PilotCommands {
    private static Pilot pilot = Robot.getPilot();

    /** Set default command to turn off the rumble */
    public static void setupDefaultCommand() {
        pilot.setDefaultCommand(rumble(0, 1)); // launchReadyRumble().withName("Pilot.default"));
    }

    /** Command that can be used to rumble the pilot controller */
    public static Command rumble(double intensity, double durationSeconds) {
        return pilot.rumbleCommand(intensity, durationSeconds);
    }

    /**
     * Command that can be used to turn on the slow mode. Slow mode modifies the fwd, left, and CCW
     * methods, we don't want these to require the pilot subsystem
     */
    public static Command slowMode() {
        return Commands.startEnd(() -> pilot.setSlowMode(true), () -> pilot.setSlowMode(false));
    }

    /**
     * Command that can be used to turn on the turbo mode. Turbo mode modifies CCW methods, we don't
     * want these to require the pilot subsystem
     */
    public static Command turboMode() {
        return Commands.startEnd(() -> pilot.setTurboMode(true), () -> pilot.setTurboMode(false));
    }
}
