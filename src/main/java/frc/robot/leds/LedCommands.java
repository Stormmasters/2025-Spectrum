package frc.robot.leds;

import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.util.Util;

public class LedCommands {
    private static Led leds = Robot.getLeds();

    static void bindTriggers() {
        disabledPattern(Util.disabled.and(Util.dsAttached));
        teleopPattern(Util.teleop.and(Util.dsAttached));
        autoPattern(Util.autoMode.and(Util.dsAttached));
        testModePattern(Util.testMode.and(Util.dsAttached));

        // Elevator Led Commands
        elevatorUpLED(Robot.getElevator().isUp().and(Util.teleop), 6);
        xButtonLED(Robot.getPilot().X, 3);
    }

    /** Default LED commands for each mode */
    private static Trigger ledDefaultCommand(String name, LEDPattern pattern, Trigger trigger) {
        int priority = -1;
        return trigger.and(leds.checkPriority(priority), leds.defaultTrigger)
                .onTrue(leds.setPattern(pattern, priority).withName(name));
    }

    static void disabledPattern(Trigger trigger) {
        ledDefaultCommand("LEDs.disabledPattern", leds.ombre(leds.purple, leds.white), trigger);
    };

    static void teleopPattern(Trigger trigger) {
        ledDefaultCommand("LEDs.teleopPattern", leds.bounce(leds.purple, 3), trigger);
    };

    static void autoPattern(Trigger trigger) {
        ledDefaultCommand(
                "LEDs.autoPattern", leds.countdown(() -> Timer.getFPGATimestamp(), 15), trigger);
    };

    static void testModePattern(Trigger trigger) {
        ledDefaultCommand("LEDs.testModePattern", leds.chase(Color.kRed, 0.2, 1), trigger);
    }

    /** LED non-default Commands, set the priority value to see which command takes precedence */
    private static Trigger ledCommand(
            String name, LEDPattern pattern, int priority, Trigger trigger) {
        return trigger.and(leds.checkPriority(priority))
                .whileTrue(leds.setPattern(pattern, priority).withName(name));
    }

    static void elevatorUpLED(Trigger trigger, int priority) {
        ledCommand("LEDs.ElevatorUp", leds.blink(Color.kBlue, 0.25), priority, trigger);
    }

    static void xButtonLED(Trigger trigger, int priority) {
        ledCommand("LEDs.XButton", leds.rainbow(), priority, trigger);
    }
}
