package frc.robot.launcher;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.launcher.Launcher.LauncherConfig;
import frc.robot.pilot.Pilot;
import java.util.function.DoubleSupplier;

public class LauncherCommands {
    private static Launcher launcher = Robot.getLauncher();
    private static LauncherConfig config = Robot.getConfig().launcher;
    private static Pilot pilot = Robot.getPilot();

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static void bindTriggers() {
        pilot.getActivate_B().whileTrue(runVelocity(config::getMaxVelocityRpm));
        pilot.getRetract_X().whileTrue(runVelocity(() -> -1 * config.getMaxVelocityRpm()));
    }

    public static Command runVelocity(DoubleSupplier velocityRPM) {
        return launcher.runVelocityTCFOCrpm(velocityRPM).withName("Launcher.runVelocity");
    }
}
