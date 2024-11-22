package frc.robot.climber;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.climber.Climber.ClimberConfig;
import frc.robot.elevator.ElevatorStates;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ClimberStates {
    private static Climber climber = Robot.getClimber();
    private static ClimberConfig config = Robot.getConfig().climber;
    private static Pilot pilot = Robot.getPilot();
    private static Operator operator = Robot.getOperator();

    public static final Trigger atMidClimbPos =
            climber.atPercentage(config::getMidClimb, config::getTolerance);
    public static final Trigger belowMidClimbPos =
            climber.belowPercentage(config::getMidClimb, config::getTolerance);
    public static final Trigger atFullExtendPos =
            climber.atPercentage(config::getFullExtend, config::getTolerance);
    public static final Trigger atHomePos =
            climber.atPercentage(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(holdPosition().ignoringDisable(true).withName("Climber.default"));
    }

    public static void setStates() {
        climbPrep.whileTrue(fullExtend());

        // What does the climber do in auto climb sequence?
        // Hooks go to mid climb, until Elevator is full up
        // Once Elevator is full up, hooks go to home
        climbRoutine.and(ElevatorStates.isUp.not()).whileTrue(midClimb());
        climbRoutine.and(ElevatorStates.isUp).whileTrue(home());
        pilot.retract_X.whileTrue(home());

        operator.topClimb_fUdp.whileTrue(fullExtend());
        operator.midClimb_fDdp.whileTrue(midClimb());
        operator.botClimb_fRdp.whileTrue(home());

        operator.safeClimb_START.whileTrue(safeClimb());
        operator.zeroClimber.whileTrue(climber.zeroClimberRoutine());
        operator.overrideClimber.whileTrue(runClimber(operator::getClimberOverride));

        coastMode.onTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
    }

    private static Command runClimber(DoubleSupplier speed) {
        return climber.runPercentage(speed).withName("Elevator.runElevator");
    }

    private static Command holdPosition() {
        return climber.holdPosition().withName("Climber.holdPosition");
    }

    private static Command fullExtend() {
        return climber.moveToPercentage(config::getFullExtend).withName("Climber.fullExtend");
    }

    private static Command home() {
        return climber.moveToPercentage(config::getHome).withName("Climber.home");
    }

    private static Command midClimb() {
        return climber.moveToPercentage(config::getMidClimb).withName("Climber.midClimb");
    }

    private static Command safeClimb() {
        return climber.moveToPercentage(config::getSafeClimb).withName("Climber.safeClimb");
    }

    private static Command coastMode() {
        return climber.coastMode().withName("Climber.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return climber.ensureBrakeMode().withName("Climber.BrakeMode");
    }

    private static Command tuneClimber() {
        return climber.moveToPercentage(new TuneValue("Tune Climber", 0).getSupplier())
                .withName("Climber.Tune");
    }
}
