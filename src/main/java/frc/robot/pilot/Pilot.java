package frc.robot.pilot;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.robot.climber.ClimberCommands;
import frc.robot.elevator.ElevatorCommands;
import frc.robot.launcher.LauncherCommands;
import frc.robot.pivot.PivotCommands;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;
import lombok.Setter;

public class Pilot extends Gamepad {
    public static class PilotConfig extends Config {

        @Getter @Setter private double slowModeScalor = 0.45;
        @Getter @Setter private double turboModeScalor = 1;

        public PilotConfig() {
            super("Pilot", 0);

            setLeftStickDeadzone(0);
            setLeftStickExp(2.0);
            setLeftStickScalor(6);

            setTriggersDeadzone(0);
            setTriggersExp(2.0);
            setTriggersScalor(3);
        }
    }

    private PilotConfig config;
    @Getter @Setter private boolean isSlowMode = false;
    @Getter @Setter private boolean isTurboMode = false;
    @Getter @Setter private boolean isFieldOriented = true;

    // Triggers
    @Getter private Trigger extend, retract;
    @Getter private Trigger intake;
    @Getter private Trigger upReorient, leftReorient, downReorient, rightReorient;

    /** Create a new Pilot with the default name and port. */
    public Pilot(PilotConfig config) {
        super(config);
        this.config = config;
        RobotTelemetry.print("Pilot Subsystem Initialized: ");
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTriggers() {
        extend = B.and(leftBumper().not(), teleop);
        intake = A.and(noBumpers(), teleop);
        retract = x().and(noBumpers(), teleop);

        // Drive Triggers
        upReorient = upDpad().and(teleop, leftBumperOnly());
        leftReorient = leftDpad().and(teleop, leftBumperOnly());
        downReorient = downDpad().and(teleop, leftBumperOnly());
        rightReorient = rightDpad().and(teleop, leftBumperOnly());

        // TEST TRIGGERS
        testMode.and(B).whileTrue(ElevatorCommands.tuneElevator());

        // OLD TRIGGERS
        x().whileTrue(ElevatorCommands.home());
        y().whileTrue(ElevatorCommands.runElevator(() -> getLeftY()));

        B.whileTrue(LauncherCommands.runVelocity(Robot.getConfig().launcher::getMaxVelocityRpm));
        x().whileTrue(
                        LauncherCommands.runVelocity(
                                () -> -1 * Robot.getConfig().launcher.getMaxVelocityRpm()));
        B.whileTrue(PivotCommands.subwoofer());
        x().whileTrue(PivotCommands.home());
        B.whileTrue(ClimberCommands.fullExtend());
        x().whileTrue(ClimberCommands.home());

        /* Use the right stick to set a cardinal direction to aim at */
        (leftBumperOnly().negate())
                .and(
                        rightXTrigger(ThresholdType.ABS_GREATER_THAN, 0.5)
                                .or(rightYTrigger(ThresholdType.ABS_GREATER_THAN, 0.5)))
                .whileTrue(PilotCommands.stickSteerDrive());
    };

    public void setMaxVelocity(double maxVelocity) {
        leftStickCurve.setScalar(maxVelocity);
    }

    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        triggersCurve.setScalar(maxRotationalVelocity);
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
        double ccwPositive = triggersCurve.calculate(getTwist());
        if (isSlowMode) {
            ccwPositive *= Math.abs(config.getSlowModeScalor());
        } else if (isTurboMode) {
            ccwPositive *= Math.abs(config.getTurboModeScalor());
        }
        return ccwPositive;
    }
}
