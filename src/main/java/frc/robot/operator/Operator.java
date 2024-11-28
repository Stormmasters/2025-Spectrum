package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;

public class Operator extends Gamepad {
    // Triggers, these would be robot states such as ampReady, intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();
    public final Trigger intake_A = A.and(noFn, teleop);
    public final Trigger eject_fA = A.and(fn, teleop);

    public final Trigger noteToAmp_B = B.and(noFn, teleop);
    public final Trigger ampEject_fB = B.and(fn, teleop);

    public final Trigger feederFwd_Y = Y.and(noFn, teleop);
    public final Trigger feederRev_fY = Y.and(fn, teleop);

    public final Trigger elevatorAmp_X = X.and(noFn, teleop);
    public final Trigger elevatorHome_fX = X.and(fn, teleop);

    public final Trigger overrideClimber = rightStickY.and(fn, teleop);
    public final Trigger overrideElevator = leftStickY.and(fn, teleop);

    public final Trigger resetPose_RBLB = bothBumpers.and(teleop);
    public final Trigger safeClimb_START = start.and(noFn, teleop);

    public final Trigger zeroClimber = select.and(noFn, teleop);
    public final Trigger zeroElevator = select.and(fn, rightBumper, teleop);
    public final Trigger zeroPivot = select.and(leftBumperOnly, teleop);

    public final Trigger increaseOffset_Udp = upDpad.and(noFn, teleop);
    public final Trigger decreaseOffset_Ddp = downDpad.and(noFn, teleop);
    public final Trigger resetOffset_Rdp = leftDpad.and(noFn, teleop);
    public final Trigger switchFeedSpot = rightDpad.and(noFn, teleop);

    // Climb
    public final Trigger topClimb_fUdp = upDpad.and(fn, teleop);
    public final Trigger midClimb_fDdp = downDpad.and(fn, teleop);
    public final Trigger elevatorFullExtend_fLdp = leftDpad.and(fn, teleop);
    public final Trigger botClimb_fRdp = rightDpad.and(fn, teleop);

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B);
    public final Trigger coastOff_dA = disabled.and(A);

    // TEST TRIGGERS

    public static class OperatorConfig extends Config {

        public OperatorConfig() {
            super("Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    private OperatorConfig config;

    public Operator(OperatorConfig config) {
        super(config);
        this.config = config;
        Robot.add(this);
        Telemetry.print("Operator Subsystem Initialized: ");
    }

    public void setupStates() {
        // Left Blank so we can bind when the controller is connected
        OperatorStates.setStates();
    }

    public void setupDefaultCommand() {
        OperatorStates.setupDefaultCommand();
    }

    public double getElevatorOverride() {
        return getLeftY();
    }

    public double getClimberOverride() {
        return getRightY();
    }
}
