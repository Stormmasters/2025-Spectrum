package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.pilot.Pilot;

/**
 * This class is used for commands that use multiple subsystems and don't directly call a gamepad.
 * This is often command groups such as moving an arm and turning on an intake, etc. In 2023 we
 * called this MechanismCommands.java
 */
public class States {
    private static final Pilot pilot = Robot.getPilot();

    // Define Robot States here and how they can be triggered
    // States should be triggers that command multiple mechanism or can be used in teleop or auton
    // Use onTrue/whileTrue to run a command when entering the state
    // Use onFalse/whileFalse to run a command when leaving the state
    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger intaking = pilot.intake_A.or(visionIntaking);

    public static final Trigger ampReady = pilot.amp_B;

    public static final Trigger score = Trigger.kFalse;

    public static final Trigger preSpeaker = Trigger.kFalse;
    public static final Trigger preSubwoofer = Trigger.kFalse;
    public static final Trigger preFeed = Trigger.kFalse;

    public static final Trigger climbReady = Trigger.kFalse;
    public static final Trigger climbAuto = Trigger.kFalse;

    public static void setupStatesTriggers() {}
}
