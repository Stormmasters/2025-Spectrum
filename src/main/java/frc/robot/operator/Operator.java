package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.RobotTelemetry;
import frc.spectrumLib.gamepads.Gamepad;
import lombok.Getter;

public class Operator extends Gamepad {
    public static class OperatorConfig extends Config {

        @Getter private final double triggersDeadzone = 0;

        public OperatorConfig() {
            super("Operator", 1);
        }
    }

    private OperatorConfig config;

    // Triggers, these would be robot states such as ampReady, intake, visionAim, subwooferShot,
    // launch, etc.
    @Getter private Trigger fn, noFn, scoreFn; // These are our function keys to overload buttons
    @Getter private Trigger intake, ejectIntake;
    @Getter private Trigger manualAmp, ejectAmp;
    @Getter private Trigger feederForward, feederBackward;
    @Getter private Trigger elevatorUp, elevatorHome;
    @Getter private Trigger coastMode;

    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;
        Robot.subsystems.add(this);
        RobotTelemetry.print("Operator Subsystem Initialized: ");
    }

    public void bindTriggers() {
        // Left Blank so we can bind when the controller is connected
    }

    public void setupDefaultCommand() {
        OperatorCommands.setupDefaultCommand();
    }

    /** Setup the Buttons for telop mode. */
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simualation */
    public void setupTriggers() {
        fn = leftBumperOnly;
        noFn = fn.not();
        scoreFn = fn.or(bothBumpers);

        intake = A.and(noFn);
        ejectIntake = A.and(fn, teleop);
        manualAmp = B.and(noFn, teleop);
        ejectAmp = B.and(fn, teleop);
        feederForward = Y.and(noFn, teleop);
        feederBackward = Y.and(fn, teleop);
        elevatorUp = X.and(noFn, teleop);
        elevatorHome = X.and(fn, teleop);

        coastMode = B.and(disabled);
        // TODO: Add manual climber and elevator
    };
}
