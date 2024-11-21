package frc.robot.fakeElevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.climber.ClimberStates;
import frc.robot.fakeElevator.FakeElevator.FakeElevatorConfig;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;

public class FakeElevatorStates {
    private static FakeElevator fakeElevator = Robot.getFakeElevator();
    private static FakeElevatorConfig config = Robot.getConfig().fakeElevator;
    private static Pilot pilot = Robot.getPilot();

    /* Check FakeElevator States */
    // Is Amp Height
    public static final Trigger isAtAmp =
            fakeElevator.atPercentage(config::getAmp, config::getTolerance);
    public static final Trigger isUp =
            fakeElevator.atPercentage(config::getElevatorUpHeight, config::getTolerance);
    public static final Trigger isHome =
            fakeElevator.atPercentage(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        fakeElevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("FakeElevator.default"));
    }

    public static void setStates() {
        // Test statements to show how these triggers work
        isAtAmp.onTrue(Commands.print("At Amp Height"));
        isUp.onTrue(Commands.print("FakeElevator Up"));

        ampPrep.whileTrue(amp());
        score.onFalse(home()); // Return home when we stop the scoring action

        // FakeElevator Extends when the climber is at mid climb
        climbRoutine.and(ClimberStates.atMidClimbPos).onTrue(fullExtend());

        // Test Mode Buttons
        pilot.tuneElevator_tB.whileTrue(tuneElevator());

        coastMode.onTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
    }

    public static Command runFakeElevator(DoubleSupplier speed) {

        return fakeElevator.runPercentage(speed).withName("FakeElevator.runFakeElevator");
    }

    public static Command holdPosition() {
        return fakeElevator.holdPosition().withName("FakeElevator.holdPosition");
    }

    public static Command fullExtend() {
        return fakeElevator
                .moveToRotations(config::getFullExtend)
                .withName("FakeElevator.fullExtend");
    }

    public static Command amp() {
        return fakeElevator.moveToRotations(config::getAmp).withName("FakeElevator.amp");
    }

    public static Command trap() {
        return fakeElevator.moveToRotations(config::getTrap).withName("FakeElevator.trap");
    }

    public static Command home() {
        return fakeElevator.moveToRotations(config::getHome).withName("FakeElevator.home");
    }

    public static Command zero() {
        return fakeElevator.zeroFakeElevatorRoutine().withName("Zero FakeElevator");
    }

    public static Command coastMode() {
        return fakeElevator.coastMode().withName("FakeElevator.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return fakeElevator.ensureBrakeMode().withName("FakeElevator.BrakeMode");
    }

    // Example of a TuneValue that is used to tune a single value in the code
    public static Command tuneElevator() {
        return fakeElevator
                .moveToRotations(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }
}
