package frc.robot.operator;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.gamepads.Gamepad;

public class PhotonOperator extends Gamepad {
    // Triggers, these would be robot states such as intake, visionAim, etc.
    // If triggers need any of the config values set them in the constructor
    /*  A, B, X, Y, Left Bumper, Right Bumper = Buttons 1 to 6 in simulation */
    public final Trigger fn = leftBumper;
    public final Trigger noFn = fn.not();

    public final Trigger leftScore_Dpad = leftDpad.and(noFn, teleop);
    public final Trigger rightScore_Dpad = rightDpad.and(noFn, teleop);

    public final Trigger climbPrep_start = start.and(noFn, teleop);

    // public final Trigger operatorCoralStage = leftBumper.and(teleop);
    // public final Trigger operatorAlgaeStage = rightBumper.and(teleop);
    // public final Trigger notStage = operatorCoralStage.not().and(operatorAlgaeStage.not());

    // public final Trigger L1Coral_A = A.and(teleop);
    public final Trigger L2Coral_B = B.and(teleop);
    public final Trigger L3Coral_X = X.and(teleop);
    public final Trigger L4Coral_Y = Y.and(teleop);

    // public final Trigger homeElevator_A = A.and(notStage, teleop);
    // public final Trigger homeInClimb_B = B.and(notStage, teleop);

    // DISABLED TRIGGERS
    public final Trigger coastOn_dB = disabled.and(B); // not being used right now
    public final Trigger coastOff_dA = disabled.and(A); // not being used right now

    // TEST TRIGGERS

    public static class PhotonOperatorConfig extends Config {

        public PhotonOperatorConfig() {
            super("Photon Operator", 1);
            setTriggersDeadzone(0.0);
        }
    }

    private PhotonOperatorConfig config;

    public PhotonOperator(PhotonOperatorConfig config) {
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
