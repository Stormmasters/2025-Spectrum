package frc.robot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotConfig.RobotType;
import frc.robot.auton.Auton;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.SpectrumState;

public class RobotStates {
    private static final RobotConfig config = Robot.getRobotConfig();
    private static final Pilot pilot = Robot.getPilot();
    private static final Auton auton = Robot.getAuton();

    // Define Robot States here and how they can be triggered
    // States should be triggers that command multiple mechanism or can be used in teleop or auton
    // Use onTrue/whileTrue to run a command when entering the state
    // Use onFalse/whileFalse to run a command when leaving the state
    // RobotType Triggers
    public static final Trigger pm = new Trigger(() -> config.getRobotType() == RobotType.PM);
    public static final Trigger am = new Trigger(() -> config.getRobotType() == RobotType.AM);
    public static final Trigger fm = new Trigger(() -> config.getRobotType() == RobotType.FM);
    public static final Trigger sim = new Trigger(() -> config.getRobotType() == RobotType.SIM);

    public static final Trigger visionIntaking = Trigger.kFalse;
    public static final Trigger intaking = pilot.intake_A.or(visionIntaking);
    public static final Trigger ejecting = Trigger.kFalse;

    public static final Trigger ampReady = pilot.amp_B;

    public static final Trigger score = Trigger.kFalse;

    public static final Trigger preSpeaker = Trigger.kFalse;
    public static final Trigger preSubwoofer = Trigger.kFalse;
    public static final Trigger preFeed = Trigger.kFalse;

    public static final Trigger climbReady = Trigger.kFalse;
    public static final Trigger climbAuto = Trigger.kFalse;

    public static final SpectrumState coast = new SpectrumState();
    public static final Trigger brake = Trigger.kFalse;

    public static void setupStates() {
        pilot.coast.onTrue(coast.toggle());
    }
}
