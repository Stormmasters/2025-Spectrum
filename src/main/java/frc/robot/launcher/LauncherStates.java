package frc.robot.launcher;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.launcher.Launcher.LauncherConfig;
import java.util.function.DoubleSupplier;

public class LauncherStates {
    private static Launcher launcher = Robot.getLauncher();
    private static LauncherConfig config = Robot.getConfig().launcher;

    public static void setupDefaultCommand() {
        launcher.setDefaultCommand(launcher.runStop());
    }

    public static void setStates() {
        intaking.whileTrue(runVelocity(config::getMaxVelocityRpm));
        ejecting.whileTrue(runVelocity(() -> -1 * config.getMaxVelocityRpm()));

        coast.get().onTrue(coastMode());
        coast.get().onFalse(ensureBrakeMode());
    }

    public static Command runVelocity(DoubleSupplier velocityRPM) {
        return launcher.runVelocityTCFOCrpm(velocityRPM).withName("Launcher.runVelocity");
    }

    public static Command coastMode() {
        return launcher.coastMode().withName("Launcher.CoastMode");
    }

    public static Command stopMotor() {
        return launcher.runStop().withName("Launcher.stop");
    }

    public static Command ensureBrakeMode() {
        return launcher.ensureBrakeMode().withName("Launcher.BrakeMode");
    }
}
