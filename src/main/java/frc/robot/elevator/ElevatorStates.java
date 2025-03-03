package frc.robot.elevator;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elevator.Elevator.ElevatorConfig;
import frc.spectrumLib.Telemetry;
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
            elevator.atRotations(config::getHome, config::getTriggerTolerance);
    public static final Trigger isL2Coral =
            elevator.atRotations(config::getL2Coral, config::getTriggerTolerance);
    public static final Trigger isL3Coral =
            elevator.atRotations(config::getL3Coral, config::getTriggerTolerance);
    public static final Trigger isL4Coral =
            elevator.atRotations(config::getL4Coral, config::getTriggerTolerance);

    public static void setupDefaultCommand() {
        elevator.setDefaultCommand(holdPosition().withName("Elevator.default"));
        // Removed run when disabled, so that the elevator doesn't jump up on enable
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        homeAll.whileTrue(home());

        // algaeHandoff.whileTrue(handOff());
        // coralHandoff.whileTrue(handOff());

        stationIntaking.whileTrue(setPosition(config::getStationIntake));
        stationExtendedIntaking.whileTrue(setPosition(config::getStationExtendedIntake));
        stationIntaking.or(stationExtendedIntaking).onFalse(home());

        Robot.getPilot().photonRemoveL2Algae.whileTrue(setPosition(config::getL2Algae));
        Robot.getPilot().photonRemoveL3Alage.whileTrue(setPosition(config::getL3Algae));
        Robot.getPilot()
                .photonRemoveL2Algae
                .or(Robot.getPilot().photonRemoveL3Alage)
                .onFalse(home());

        L1Coral.and(actionPrepState).whileTrue(setPosition(config::getL1Coral));
        L1Coral.and(actionState).whileTrue(setPosition(config::getL1CoralScore));
        L2Coral.and(actionPrepState).whileTrue(setPosition(config::getL2Coral));
        L2Coral.and(actionState).whileTrue(setPosition(config::getL2CoralScore));
        L3Coral.and(actionPrepState).whileTrue(setPosition(config::getL3Coral));
        L3Coral.and(actionState).whileTrue(setPosition(config::getL3CoralScore));
        L4Coral.and(actionPrepState).whileTrue(setPosition(config::getL4Coral));
        L4Coral.and(actionState).whileTrue(setPosition(config::getL4CoralScore));

        processorAlgae.and(actionPrepState).whileTrue(setPosition(config::getL1Algae));
        processorAlgae.and(actionState).whileTrue(setPosition(config::getL1AlgaeScore));
        L2Algae.and(actionPrepState).whileTrue(setPosition(config::getL2Algae));
        L2Algae.and(actionState).whileTrue(setPosition(config::getL2AlgaeScore));
        L3Algae.and(actionPrepState).whileTrue(setPosition(config::getL3Algae));
        L3Algae.and(actionState).whileTrue(setPosition(config::getL3AlgaeScore));
        netAlgae.and(actionPrepState).whileTrue(setPosition(config::getL4Algae));
        netAlgae.and(actionState).whileTrue(setPosition(config::getL4AlgaeScore));

        Robot.getPilot()
                .reZero_start
                .onTrue(elevator.resetToInitialPos()); // TODO: check if this works
    }

    private static Command runElevator(DoubleSupplier speed) {
        return elevator.runPercentage(speed).withName("Elevator.runElevator");
    }

    public static DoubleSupplier getPosition() {
        return () -> elevator.getPositionRotations();
    }

    // TODO: Remake this method
    // public static DoubleSupplier getElbowShoulderPos() {
    //     double eToSratio = 2.0; // get actual elbow to shoulder length ratio
    //     double e = Math.abs(Elbow.getPosition().getAsDouble());
    //     double s = 100 - Math.abs(ShoulderStates.getPosition().getAsDouble());
    //     return () -> (eToSratio * e + s) / 3;
    // }

    // public static boolean allowedPosition() {
    //     return ((getPosition().getAsDouble() * 100 / config.getL4Coral() + 10)
    //                     - getElbowShoulderPos().getAsDouble())
    //             > 0;
    // }

    private static Command setPosition(DoubleSupplier position) {
        return elevator.setPosition(position);
    }

    private static Command holdPosition() {
        return elevator.holdPosition().withName("Elevator.holdPosition");
    }

    private static Command home() {
        return elevator.setPosition(config::getHome).withName("Elevator.home");
    }

    private static Command coastMode() {
        return elevator.coastMode().withName("Elevator.CoastMode");
    }

    private static Command ensureBrakeMode() {
        return elevator.ensureBrakeMode().withName("Elevator.BrakeMode");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
