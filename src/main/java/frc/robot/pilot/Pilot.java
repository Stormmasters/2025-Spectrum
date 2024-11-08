package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {
    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double defaultTurnScalor = 0.75;
        @Getter @Setter private double turboModeScalor = 1;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(0);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setRightStickDeadzone(0);
            setRightStickExp(2.0);
            setRightStickScalor(3);

            setTriggersDeadzone(0);
            setTriggersExp(1);
            setTriggersScalor(1);
        }
    }

    private PilotConfig config;
    @Getter @Setter private boolean isSlowMode = false;
    @Getter @Setter private boolean isTurboMode = false;
    @Getter @Setter private boolean isFieldOriented = true;

    // Triggers, these would be robot states such as ampReady, intake, visionAim, subwooferShot,
    // launch, etc.
    @Getter private Trigger fn, noFn, scoreFn; // These are our function keys to overload buttons
    @Getter private Trigger activate_B, retract_X;
    @Getter private Trigger intake_A;
    @Getter private Trigger manual_Y;
    @Getter private Trigger upReorient, leftReorient, downReorient, rightReorient;
    @Getter private Trigger stickSteer;
    @Getter private Trigger tuneElevator;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;
        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTriggers() {
        fn = leftBumperOnly;
        noFn = fn.not();
        scoreFn = fn.or(bothBumpers);

        intake_A = A.and(noFn, teleop);
        activate_B = B.and(noFn, teleop);
        retract_X = X.and(noFn, teleop);
        manual_Y = Y.and(noFn, teleop);

        // Drive Triggers
        upReorient = upDpad.and(fn, teleop);
        leftReorient = leftDpad.and(fn, teleop);
        downReorient = downDpad.and(fn, teleop);
        rightReorient = rightDpad.and(fn, teleop);

        // TEST TRIGGERS
        tuneElevator = testMode.and(B);

        /* Use the right stick to set a cardinal direction to aim at */
        stickSteer =
                fn.and(
                        rightXTrigger(Threshold.ABS_GREATER, 0.5)
                                .or(rightYTrigger(Threshold.ABS_GREATER, 0.5)));
    };

    // DRIVE METHODS
    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        rightStickCurve.setScalar(maxRotationalVelocity);
    }

    // Positive is forward, up on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveFwdPositive() {
        double fwdPositive = leftStickCurve.calculate(-1 * getLeftY());
        if (isSlowMode) {
            fwdPositive *= Math.abs(config.getSlowModeScalor());
        }
        return fwdPositive;
    }

    // Positive is left, left on the left stick is positive
    // Applies Expontial Curve, Deadzone, and Slow Mode toggle
    public double getDriveLeftPositive() {
        double leftPositive = -1 * leftStickCurve.calculate(getLeftX());
        if (isSlowMode) {
            leftPositive *= Math.abs(config.getSlowModeScalor());
        }
        return leftPositive;
    }

    // Positive is counter-clockwise, left Trigger is positive
    // Applies Exponential Curve, Deadzone, and Slow Mode toggle
    public double getDriveCCWPositive() {
        double ccwPositive = rightStickCurve.calculate(getRightX());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        } else {
            ccwPositive *= Math.abs(config.getDefaultTurnScalor());
        }
        return ccwPositive;
    }

    // ELEVATOR METHODS
    public double getElevatorManualAxis() {
        return getLeftY();
    }
}
