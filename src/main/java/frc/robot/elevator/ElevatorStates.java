package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.climber.ClimberStates;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;
    private static Pilot pilot = Robot.getPilot();

    /* Check Elevator States */
    // Is Amp Height
    public static final Trigger isAtAmp =
            elevator.atPercentage(config::getAmp, config::getTolerance);
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorUpHeight, config::getTolerance);
    public static final Trigger isHome =
            elevator.atPercentage(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static void setStates() {
        // Test statements to show how these triggers work
        isAtAmp.onTrue(Commands.print("At Amp Height"));
        isUp.onTrue(Commands.print("Elevator Up"));

        ampPrep.whileTrue(amp());
        score.onFalse(home()); // Return home when we stop the scoring action

        // Elevator Extends when the climber is at mid climb
        climbRoutine.and(ClimberStates.atMidClimbPos).onTrue(fullExtend());

        // Test Mode Buttons
        pilot.tuneElevator_tB.whileTrue(tuneElevator());

        coastMode.onTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
    }

    public static Command runElevator(DoubleSupplier speed) {

        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    public static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    public static Command fullExtend() {
        return elevator.moveToRotations(config::getFullExtend).withName("Elevator.fullExtend");
    }

    public static Command amp() {
        return elevator.moveToRotations(config::getAmp).withName("Elevator.amp");
    }

    public static Command trap() {
        return elevator.moveToRotations(config::getTrap).withName("Elevator.trap");
    }

    public static Command home() {
        return elevator.moveToRotations(config::getHome).withName("Elevator.home");
    }

    public static Command zero() {
        return elevator.zeroElevatorRoutine().withName("Zero Elevator");
    }

    public static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    public static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Example of a TuneValue that is used to tune a single value in the code
    public static Command tuneElevator() {
        return elevator.moveToRotations(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }
}
