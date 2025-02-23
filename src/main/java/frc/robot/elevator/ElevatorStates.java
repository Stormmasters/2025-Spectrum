package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;

    /* Check Elevator States */
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorIsUpHeight, config::getTriggerTolerance);
    public static final Trigger isHigh =
            elevator.atPercentage(config::getElevatorIsHighHeight, config::getTriggerTolerance);
    public static final Trigger isHome =
            elevator.atPercentage(config::getHome, config::getTriggerTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(holdPosition().withName("Elevator.default"));
        // Removed run when disabled, so that the elevator doesn't jump up on enable
    }

    public static void setStates() {
        // Elevator Extends when the climber is at mid climb
        // Test Mode Buttons
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        // TODO: Reenable other states
        // scoreState.onTrue(score());

        // algaeHandoff.whileTrue(handOff());
        // coralHandoff.whileTrue(handOff());

        // stationIntaking.whileTrue(stationIntake());
        // stationExtendedIntake.whileTrue(stationExtendedIntake());
        // L2Algae.whileTrue(l1());
        // L3Algae.whileTrue(l1());
        // barge.whileTrue(l1());

        // L1Coral.whileTrue(l1());
        // L2Coral.whileTrue(l1());
        // L3Coral.whileTrue(l1());
        // L4Coral.whileTrue(l1());

        // actionPrepState.and(L1Coral).whileTrue(l1());
        // actionPrepState
        //         .and(L2Coral.not())
        //         .whileTrue(l2Coral()); // TODO: Remove not used for twisting
        // actionPrepState.and(L3Coral).whileTrue(l3Coral());
        // actionPrepState.and(L4Coral).whileTrue(l4());

        // actionPrepState.and(L2Algae).whileTrue(l2Algae());
        // actionPrepState.and(L3Algae).whileTrue(l3Algae());
        // actionPrepState.and(barge).whileTrue(barge());

        // homeAll.whileTrue(home());
        // homeElevator.whileTrue(zero());

        // // TODO: For Testing
        Robot.getPilot()
                .testTune_tA
                .whileTrue(elevator.setElevatorMMPositionFOC((config::getStationIntake)));
        // Robot.getPilot()
        //         .testTune_tB
        //         .whileTrue(elevator.setElevatorMMPositionFOC(config::getL3Algae));
        // Robot.getPilot().testTune_tX.whileTrue(elevator.setElevatorMMPositionFOC(config::getHome));
        // Robot.getPilot()
        //         .testTune_tY
        //         .whileTrue(elevator.setElevatorMMPositionFOC(config::getL2Coral));
        Robot.getPilot().reZero_start.onTrue(elevator.resetToIntialPos());
        // Robot.getPilot()
        //         .testTriggersTrigger
        //         .whileTrue(runElevator(() -> Robot.getPilot().getTestTriggersAxis()));
        Robot.getOperator().test_tA.whileTrue(elevator.moveToDegrees(config::getL1));
        homeAll.whileTrue(home());
    }

    private static Command runElevator(DoubleSupplier speed) {
        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    public static DoubleSupplier getPosition() {
        return () -> elevator.getPositionRotations();
    }

    public static DoubleSupplier getElbowShoulderPos() {
        double eToSratio = 2.0; // get actual elbow to shoulder length ratio
        double e = Math.abs(ElbowStates.getPosition().getAsDouble());
        double s = 100 - Math.abs(ShoulderStates.getPosition().getAsDouble());
        return () -> (eToSratio * e + s) / 3;
    }

    public static boolean allowedPosition() {
        if ((getPosition().getAsDouble() * 100 / config.getL4() + 10)
                        - getElbowShoulderPos().getAsDouble()
                > 0) {
            return true;
        } else {
            return false;
        }
    }

    private static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command fullExtend() {
        return elevator.moveToRotations(config::getFullExtend).withName("Elevator.fullExtend");
    }

    private static Command score() {
        return elevator.moveToRelativePosition(() -> -2).withName("Elbow.score");
    }

    private static Command home() {
        return elevator.moveToRotations(config::getHome)
                .alongWith(elevator.checkMaxCurrent(() -> 100))
                .withName("Elevator.home");
    }

    private static Command handOff() {
        return elevator.moveToRotations(config::getHandOff).withName("Elevator.handOff");
    }

    private static Command l2Algae() {
        return elevator.moveToRotations(config::getL2Algae).withName("Elevator.l2Algae");
    }

    private static Command l3Algae() {
        return elevator.moveToRotations(config::getL3Algae).withName("Elevator.l3Algae");
    }

    private static Command l1() {
        return elevator.moveToRotations(config::getL1).withName("Elevator.l1");
    }

    private static Command l2Coral() {
        return elevator.moveToRotations(config::getL2Coral).withName("Elevator.l2Coral");
    }

    private static Command l3Coral() {
        return elevator.moveToRotations(config::getL3Coral).withName("Elevator.l3Coral");
    }

    private static Command l4() {
        return elevator.moveToRotations(config::getL4).withName("Elevator.l4");
    }

    private static Command barge() {
        return elevator.moveToRotations(config::getBarge).withName("Elevator.barge");
    }

    private static Command stationIntake() {
        return elevator.moveToRotations(config::getStationIntake)
                .withName("Elevator.stationIntake");
    }

    private static Command stationExtendedIntake() {
        return elevator.moveToRotations(config::getStationExtendedIntake)
                .withName("Elevator.stationExtendedIntake");
    }

    private static Command zero() {
        return elevator.zeroElevatorRoutine().withName("Zero Elevator");
    }

    private static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Example of a TuneValue that is used to tune a single value in the code
    private static Command tuneElevator() {
        return elevator.moveToRotations(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
