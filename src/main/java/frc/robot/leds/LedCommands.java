package frc.robot.leds;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.spectrumLib.util.Util;

public class LedCommands {
    private static Led leds = Robot.getLeds();

    static void bindTriggers() {
        leds.defaultTrigger.and(Util.disabled, Util.dsAttached).onTrue(disabledPattern());
        leds.defaultTrigger.and(Util.teleop).onTrue(teleopPattern());
        leds.defaultTrigger.and(Util.autoMode).onTrue(autoPattern());
        leds.defaultTrigger.and(Util.testMode).onTrue(testModePattern());

        // Elevator Led Commands
        Robot.getElevator().isUp().and(Util.teleop).whileTrue(LedCommands.elevatorUpPattern());
        Robot.getPilot().X.and(Util.teleop).whileTrue(leds.setPattern(leds.rainbow(), 3));
    }

    static Command disabledPattern() {
        return leds.setPattern(leds.ombre(leds.purple, leds.white), -1)
                .withName("LEDs.disabledPattern");
    };

    static Command teleopPattern() {
        return leds.setPattern(leds.bounce(leds.purple, 3), -1).withName("LEDs.teleopPattern");
    };

    static Command autoPattern() {
        return leds.setPattern(leds.countdown(() -> Timer.getFPGATimestamp(), 15), -1)
                .withName("LEDs.autoPattern");
    };

    static Command testModePattern() {
        return leds.setPattern(leds.chase(Color.kRed, 0.2, 1), -1).withName("LEDs.testModePattern");
    }

    static Command elevatorUpPattern() {
        return leds.setPattern(leds.blink(Color.kBlue, 0.25), 6).withName("LEDs.elevatorUpPattern");
    }
}
