package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.SpectrumState;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;
import frc.spectrumLib.util.Util;
import lombok.Getter;
import lombok.Setter;

public class PhotonPilot extends Gamepad {

    // Triggers, these would be robot states such as ampReady, intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();

    // Drive Triggers
    public final Trigger upReorient = upDpad.and(fn, teleop);
    public final Trigger leftReorient = leftDpad.and(fn, teleop);
    public final Trigger downReorient = downDpad.and(fn, teleop);
    public final Trigger rightReorient = rightDpad.and(fn, teleop);

    /* Use the right stick to set a cardinal direction to aim at */
    public final Trigger driving;
    public final Trigger steer;

    public final Trigger snapSteer = Trigger.kFalse;

    public final Trigger fpv_rs = rightStickClick.and(teleop); // Remapped to Right back button

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    // TEST TRIGGERS
    public final Trigger testTune_tB = testMode.and(B);
    public final Trigger testTune_tA = testMode.and(A);
    public final Trigger testTune_tX = testMode.and(X);
    public final Trigger testTune_tY = testMode.and(Y);
    public final Trigger testTune_RB = testMode.and(rightBumper);
    public final Trigger testTune_LB = testMode.and(leftBumper);
    public final Trigger testTriggersTrigger = testMode.and(leftTrigger.or(rightTrigger));

    public static class PhotonPilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.75;
        @Getter @Setter private double turboModeScalor = 1;
        private double deadzone = 0.001;

        public PhotonPilotConfig() {
            super("Photon Pilot", 0);

            setLeftStickDeadzone(deadzone);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setRightStickDeadzone(deadzone);
            setRightStickExp(2.0);
            setRightStickScalor(12);

            setTriggersDeadzone(deadzone);
            setTriggersExp(1);
            setTriggersScalor(1);
        }
    }

    private PhotonPilotConfig config;

    private @Getter @Setter SpectrumState slowMode = new SpectrumState("SlowMode");

    @Getter @Setter SpectrumState turboMode = new SpectrumState("TurboMode");

    /** Create a new Pilot with the default name and port. */
    public PhotonPilot(PhotonPilotConfig config) {
        super(config);
        this.config = config;
        Robot.add(this);

        driving = Util.teleop.and(leftStickX.or(leftStickY));
        steer = Util.teleop.and(rightStickX.or(rightStickY));

        Telemetry.print("Pilot Subsystem Initialized: ");
    }

    public void setupStates() {
        // Used for setting rumble and control mode states only
        PilotStates.setStates();
    }

    public void setupDefaultCommand() {
        PilotStates.setupDefaultCommand();
    }

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (slowMode.getAsBoolean()) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (slowMode.getAsBoolean()) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (slowMode.getAsBoolean()) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (turboMode.getAsBoolean()) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return -1 * ccwPositive; // invert the value
    }

    public double getTestTriggersAxis() { // TODO: Remove after Testing
        return getRightTriggerAxis() - getLeftTriggerAxis();
    }
}
