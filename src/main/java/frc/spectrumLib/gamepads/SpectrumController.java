package frc.spectrumLib.gamepads;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * Handles the conversion from Xbox controller to PS5 Controller. Basic control of the controllers.
 * Do not put modifiers or custom triggers in here. use {@link Gamepad} instead
 */
public class SpectrumController {
    private boolean attached;
    private CommandXboxController xboxController;
    public Trigger A = Trigger.kFalse;
    public Trigger B = Trigger.kFalse;

    public SpectrumController(int port, boolean attached) {
        this.attached = attached;
        if (attached) {
            xboxController = new CommandXboxController(port);
            A = xboxController.a();
            B = xboxController.b();
        }
    }

    public boolean isConnected() {
        if (attached) {
            return this.getHID().isConnected();
        } else {
            return false;
        }
    }

    /* Basic Controller Buttons */

    public Trigger x() {
        return xboxController.x();
    }

    public Trigger y() {
        return xboxController.y();
    }

    public Trigger leftBumper() {
        return xboxController.leftBumper();
    }

    public Trigger rightBumper() {
        return xboxController.rightBumper();
    }

    public Trigger leftTrigger(double threshold) {
        return xboxController.leftTrigger(threshold);
    }

    public Trigger rightTrigger(double threshold) {
        return xboxController.rightTrigger(threshold);
    }

    public Trigger leftStick() {
        return xboxController.leftStick();
    }

    public Trigger rightStick() {
        return xboxController.rightStick();
    }

    public Trigger start() {
        return xboxController.start();
    }

    public Trigger select() {
        return xboxController.back();
    }

    public Trigger upDpad() {
        return xboxController.povUp();
    }

    public Trigger downDpad() {
        return xboxController.povDown();
    }

    public Trigger leftDpad() {
        return xboxController.povLeft();
    }

    public Trigger rightDpad() {
        return xboxController.povRight();
    }

    public double getRightTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightTriggerAxis();
    }

    public double getLeftTriggerAxis() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftTriggerAxis();
    }

    public double getTwist() {
        double right = getRightTriggerAxis();
        double left = getLeftTriggerAxis();
        double value = right - left;
        return value;
    }

    public double getLeftX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftX();
    }

    public double getLeftY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getLeftY();
    }

    public double getRightX() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightX();
    }

    public double getRightY() {
        if (!isConnected()) {
            return 0.0;
        }
        return xboxController.getRightY();
    }

    public GenericHID getHID() {
        if (!attached) {
            return null;
        }
        return xboxController.getHID();
    }

    public GenericHID getRumbleHID() {
        if (!isConnected()) {
            return null;
        }
        return xboxController.getHID();
    }

    public void rumbleController(double leftIntensity, double rightIntensity) {
        if (!isConnected()) {
            return;
        }
        getRumbleHID().setRumble(RumbleType.kLeftRumble, leftIntensity);
        getRumbleHID().setRumble(RumbleType.kRightRumble, rightIntensity);
    }
}
