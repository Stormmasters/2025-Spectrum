package frc.robot.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.RobotCommands;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.leds.LEDsCommands;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.mechanism.MechanismCommands;
import frc.spectrumLib.util.TuneValue;
import frc.spectrumLib.util.Util;
import java.util.function.DoubleSupplier;

public class ElevatorCommands extends MechanismCommands {
    private Elevator elevator = Robot.getElevator();
    private ElevatorConfig config = Robot.getConfig().elevator;
    private Pilot pilot = Robot.getPilot();

    protected ElevatorCommands() {
        elevator = Robot.getElevator();
        pilot = Robot.getPilot();
        RobotCommands.addMechanismCommand(this);
    };

    public void setupDefaultCommand() {
        Robot.getElevator()
                .setDefaultCommand(
                        holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public void bindTriggers() {
        pilot = Robot.getPilot();
        pilot.getActivate_B().whileTrue(fullExtend());
        pilot.getRetract_X().whileTrue(home());
        pilot.getManual_Y().whileTrue(runElevator(pilot::getElevatorManualAxis));

        // LED Commands
        elevator.isUp().and(Util.teleop).whileTrue(LEDsCommands.solidOrangeLED());

        // Test Mode Buttons
        pilot.getTuneElevator().whileTrue(tuneElevator());
    }

    protected Command runElevator(DoubleSupplier speed) {

        elevator = Robot.getElevator();
        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    protected Command holdPosition() {
        return Robot.getElevator().holdPosition().withName("Elevator.holdPosition");
    }

    protected Command fullExtend() {
        return elevator.moveToPoseRevolutions(config::getFullExtend)
                .withName("Elevator.fullExtend");
    }

    protected Command amp() {
        return elevator.moveToPoseRevolutions(config::getAmp).withName("Elevator.amp");
    }

    protected Command trap() {
        return elevator.moveToPoseRevolutions(config::getTrap).withName("Elevator.trap");
    }

    protected Command home() {
        return elevator.moveToPoseRevolutions(config::getHome).withName("Elevator.home");
    }

    protected Command zero() {
        return elevator.zeroElevatorRoutine().withName("Zero Elevator");
    }

    public Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    public Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Example of a TuneValue that is used to tune a single value in the code
    protected Command tuneElevator() {
        return elevator.moveToPoseRevolutions(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }
}
