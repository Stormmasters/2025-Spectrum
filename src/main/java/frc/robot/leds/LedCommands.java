package frc.robot.leds;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.leds.Led.LedConfig;
import frc.spectrumLib.util.Util;

public class LedCommands {
    private static Led leds = Robot.getLeds();
    private static LedConfig config = Robot.getConfig().leds;

    public static void bindTriggers() {
        leds.defaultTrigger.and(Util.disabled, Util.dsAttached).onTrue(disabledPattern());
        leds.defaultTrigger.and(Util.teleop).onTrue(teleopPattern());
        leds.defaultTrigger.and(Util.autoMode).onTrue(autoPattern());
        leds.defaultTrigger.and(Util.testMode).onTrue(testModePattern());

        // Elevator Led Commands
        Robot.getElevator().isUp().and(Util.teleop).whileTrue(LedCommands.elevatorUpPattern());
    }

    static Command disabledPattern() {
        return leds.setPattern(leds.ombre(config.getSPECTRUM_COLOR(), Color.kWhite))
                .withName("LEDs.disabledPattern");
    };

    static Command teleopPattern() {
        return leds.setPattern(leds.bounce(config.getSPECTRUM_COLOR(), 3))
                .withName("LEDs.teleopPattern");
    };

    static Command autoPattern() {
        return leds.setPattern(leds.countdown(() -> Timer.getFPGATimestamp(), 15))
                .withName("LEDs.autoPattern");
    };

    static Command testModePattern() {
        return leds.setPattern(leds.chase(Color.kRed, 0.2, 1)).withName("LEDs.testModePattern");
    }

    static Command elevatorUpPattern() {
        return leds.setPattern(LEDPattern.solid(Color.kOrange).blink(Seconds.of(0.25)))
                .withName("LEDs.elevatorUpPattern");
    }
}
