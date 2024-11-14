package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.util.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;
    private static Pilot pilot = Robot.getPilot();

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static void setStates() {
        // pilot = Robot.getPilot();
        ampPrep.whileTrue(fullExtend());
        score.onFalse(home()); // Return home whne we stop the scoring action
        // pilot.manual_Y.whileTrue(runElevator(pilot::getElevatorManualAxis));

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
        return elevator.moveToPoseRevolutions(config::getFullExtend)
                .withName("Elevator.fullExtend");
    }

    public static Command amp() {
        return elevator.moveToPoseRevolutions(config::getAmp).withName("Elevator.amp");
    }

    public static Command trap() {
        return elevator.moveToPoseRevolutions(config::getTrap).withName("Elevator.trap");
    }

    public static Command home() {
        return elevator.moveToPoseRevolutions(config::getHome).withName("Elevator.home");
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
        return elevator.moveToPoseRevolutions(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }
}
