package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.ElbowStates;
import frc.robot.elevator.ElevatorLeft.ElevatorLeftConfig;
import frc.robot.elevator.ElevatorRight.ElevatorRightConfig;
import frc.robot.shoulder.ShoulderStates;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class ElevatorStates {
    private static ElevatorLeft elevatorLeft = Robot.getElevatorLeft();
    private static ElevatorRight elevatorRight = Robot.getElevatorRight();
    private static ElevatorLeftConfig configLeft = Robot.getConfig().elevatorLeft;
    private static ElevatorRightConfig configRight = Robot.getConfig().elevatorRight;

    /* Check Elevator States */
    public static final Trigger isLeftUp =
            elevatorLeft.atPercentage(
                    configLeft::getElevatorLeftUpHeight, configLeft::getTolerance);
    public static final Trigger isLeftHome =
            elevatorLeft.atPercentage(configLeft::getHome, configLeft::getTolerance);
    public static final Trigger isRightUp =
            elevatorRight.atPercentage(
                    configRight::getElevatorRightUpHeight, configRight::getTolerance);
    public static final Trigger isRightHome =
            elevatorRight.atPercentage(configRight::getHome, configRight::getTolerance);

    public static void setupDefaultCommand() {
        elevatorLeft.setDefaultCommand(
                holdLeftPosition().ignoringDisable(true).withName("ElevatorLeft.default"));
        elevatorRight.setDefaultCommand(
                holdRightPosition().ignoringDisable(true).withName("ElevatorRight.default"));
    }

    public static void setStates() {
        // Elevator Extends when the climber is at mid climb
        // Test Mode Buttons
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        scoreState.onTrue(score());

        algaeHandoff.whileTrue(handOff());
        coralHandoff.whileTrue(handOff());

        stationIntaking.whileTrue(stationIntake());
        stationExtendedIntake.whileTrue(stationExtendedIntake());
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
        actionPrepState.and(barge).whileTrue(barge());

        homeAll.whileTrue(home());
        homeElevator.whileTrue(zero());
    }

    private static Command runElevator(DoubleSupplier speed) {
        return elevatorLeft
                .runPercentage(speed)
                .withName("Elevator.runElevator")
                .alongWith(elevatorRight.runPercentage(speed));
    }

    public static DoubleSupplier getPosition() {
        return () ->
                (elevatorLeft.getPositionRotations() + elevatorRight.getPositionRotations()) / 2;
    }

    public static DoubleSupplier getElbowShoulderPos() {
        double eToSratio = 2.0; // get actual elbow to shoulder length ratio
        double e = Math.abs(ElbowStates.getPosition().getAsDouble());
        double s = 100 - Math.abs(ShoulderStates.getPosition().getAsDouble());
        return () -> (eToSratio * e + s) / 3;
    }

    public static boolean allowedPosition() {
        if ((getPosition().getAsDouble() * 100 / configLeft.getL2())
                        - getElbowShoulderPos().getAsDouble()
                > 0) {
            return true;
        } else {
            return false;
        }
    }

    private static Command holdLeftPosition() {
        return elevatorLeft.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command holdRightPosition() {
        return elevatorRight.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command fullExtend() {
        return elevatorLeft
                .moveToRotations(configLeft::getFullExtend)
                .withName("Elevator.fullExtend")
                .alongWith(elevatorRight.moveToRotations(configRight::getFullExtend));
    }

    private static Command score() {
        return new ProxyCommand(
                () -> {
                    double originalPosition = ElevatorStates.getPosition().getAsDouble() - 10;
                    return elevatorLeft
                            .moveToPercentage(() -> originalPosition)
                            .alongWith(elevatorRight.moveToPercentage(() -> originalPosition))
                            .withName("Elevator.score");
                });
    }

    private static Command home() {
        return elevatorLeft
                .moveToRotations(configLeft::getHome)
                .alongWith(elevatorLeft.checkMaxCurrent(() -> 100))
                .withName("Elevator.home")
                .alongWith(elevatorRight.moveToRotations(configRight::getHome))
                .alongWith(elevatorRight.checkMaxCurrent(() -> 100));
    }

    private static Command handOff() {
        return elevatorLeft
                .moveToRotations(configLeft::getL3)
                .alongWith(elevatorRight.moveToRotations(configRight::getL3))
                .withName("Elevator.handOffUp")
                .until(() -> ElbowStates.getPosition().getAsDouble() > 90.0)
                .andThen(
                        elevatorLeft
                                .moveToRotations(configLeft::getL2)
                                .alongWith(elevatorRight.moveToRotations(configRight::getL2)))
                .withName("Elevator.handOffDown");
    }

    private static Command stationIntake() {
        return elevatorLeft
                .moveToRotations(configLeft::getStationIntake)
                .withName("Elevator.stationIntake")
                .alongWith(elevatorRight.moveToRotations(configRight::getStationIntake));
    }

    private static Command stationExtendedIntake() {
        return elevatorLeft
                .moveToRotations(configLeft::getStationExtendedIntake)
                .withName("Elevator.stationExtendedIntake")
                .alongWith(elevatorRight.moveToRotations(configRight::getStationExtendedIntake));
    }

    private static Command l1() {
        return elevatorLeft
                .moveToRotations(configLeft::getL1)
                .withName("Elevator.l1")
                .alongWith(elevatorRight.moveToRotations(configRight::getL1));
    }

    private static Command l2() {
        return elevatorLeft
                .moveToRotations(configLeft::getL2)
                .withName("Elevator.l2")
                .alongWith(elevatorRight.moveToRotations(configRight::getL2));
    }

    private static Command l3() {
        return elevatorLeft
                .moveToRotations(configLeft::getL3)
                .withName("Elevator.l3")
                .alongWith(elevatorRight.moveToRotations(configRight::getL3));
    }

    private static Command l4() {
        return elevatorLeft
                .moveToRotations(configLeft::getL4)
                .withName("Elevator.l4")
                .alongWith(elevatorRight.moveToRotations(configRight::getL4));
    }

    private static Command barge() {
        return elevatorLeft
                .moveToRotations(configLeft::getBarge)
                .withName("Elevator.barge")
                .alongWith(elevatorRight.moveToRotations(configRight::getBarge));
    }

    private static Command zero() {
        return elevatorLeft
                .zeroElevatorLeftRoutine()
                .withName("Zero Elevator")
                .alongWith(elevatorRight.zeroElevatorRightRoutine());
    }

    private static Command coastMode() {
        return elevatorLeft
                .coastMode()
                .withName("Elevator.CoastMode")
                .alongWith(elevatorRight.coastMode());
    }

    private static Command ensureBrakeMode() {
        return elevatorLeft
                .ensureBrakeMode()
                .withName("Elevator.BrakeMode")
                .alongWith(elevatorRight.ensureBrakeMode());
    }

    // Example of a TuneValue that is used to tune a single value in the code
    private static Command tuneElevator() {
        return elevatorLeft
                .moveToRotations(new TuneValue("Tune Elevator", 0).getSupplier())
                .withName("Elevator.Tune")
                .alongWith(
                        elevatorRight.moveToRotations(
                                new TuneValue("Tune Elevator", 0).getSupplier()));
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
