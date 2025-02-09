package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.robot.operator.Operator;
import frc.robot.pilot.Pilot;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static Elevator elevator = Robot.getElevator();
    private static ElevatorConfig config = Robot.getConfig().elevator;
    private static Pilot pilot = Robot.getPilot();
    private static Operator operator = Robot.getOperator();

    /* Check Elevator States */
    public static final Trigger isUp =
            elevator.atPercentage(config::getElevatorUpHeight, config::getTolerance);
    public static final Trigger isHome =
            elevator.atPercentage(config::getHome, config::getTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(
                holdPosition().ignoringDisable(true).withName("Elevator.default"));
    }

    public static void setStates() {
        // Elevator Extends when the climber is at mid climb
        // Test Mode Buttons
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        scoreState.whileTrue(score());

        algaeHandoff.whileTrue(handOff());
        coralHandoff.whileTrue(handOff());

        stationIntaking.whileTrue(log(stationIntake()));
        L2Algae.whileTrue(l1());
        L3Algae.whileTrue(l1());
        barge.whileTrue(l1());

        L1Coral.whileTrue(l1());
        L2Coral.whileTrue(l1());
        L3Coral.whileTrue(l1());
        L4Coral.whileTrue(l1());

        actionPrepState.and(L1Coral).whileTrue(l1());
        actionPrepState.and(L2Coral).whileTrue(l2());
        actionPrepState.and(L3Coral).whileTrue(l3());
        actionPrepState.and(L4Coral).whileTrue(l4());

        actionPrepState.and(L2Algae).whileTrue(l2());
        actionPrepState.and(L3Algae).whileTrue(l3());
        actionPrepState.and(barge).whileTrue(l4());

        homeAll.whileTrue(home());
        homeElevator.whileTrue(zero());
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
        if ((getPosition().getAsDouble() * 100 / config.getL2())
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
        return new ProxyCommand(
                () -> {
                    double originalPosition = ElevatorStates.getPosition().getAsDouble() - 10;
                    return elevator.moveToPercentage(() -> originalPosition)
                            .withName("Elevator.score");
                });
    }

    private static Command home() {
        return elevator.moveToRotations(config::getHome)
                .alongWith(elevator.checkMaxCurrent(() -> 100))
                .withName("Elevator.home");
    }

    private static Command handOff() {
        return elevator.moveToRotations(config::getL3)
                .withName("Elevator.handOffUp")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 90.0)
                .andThen(elevator.moveToRotations(config::getL2))
                .withName("Elevator.handOffDown");
    }

    private static Command stationIntake() {
        return elevator.moveToRotations(config::getStationIntake)
                .withName("Elevator.stationIntake");
    }

    private static Command l1() {
        return elevator.moveToRotations(config::getL1).withName("Elevator.l1");
    }

    private static Command l2() {
        return elevator.moveToRotations(config::getL2).withName("Elevator.l2");
    }

    private static Command l3() {
        return elevator.moveToRotations(config::getL3).withName("Elevator.l3");
    }

    private static Command l4() {
        return elevator.moveToRotations(config::getL4).withName("Elevator.l4");
    }

    private static Command barge() {
        return elevator.moveToRotations(config::getBarge).withName("Elevator.barge");
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
