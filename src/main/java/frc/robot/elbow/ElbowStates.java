package frc.robot.elbow;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.elbow.Elbow.ElbowConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ElbowStates {
    private static Elbow elbow = Robot.getElbow();
    private static ElbowConfig config = Robot.getConfig().elbow;

    public static final Trigger isHome = elbow.atPercentage(config::getHome, config::getTolerance);
    public static final Trigger pastElevator = elbow.aboveDegrees(elbow.offsetPosition(() -> -160 + 360), config::getTolerance).and(elbow.belowDegrees(() -> 160, config::getTolerance));

    public static void setupDefaultCommand() {
        elbow.setDefaultCommand(log(elbow.runHoldElbow().withName("Elbow.default")));
        // elbow.runStop());
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        // TODO: Uncomment after Testing
        // stationIntaking.and(backwardMode.not()).whileTrue(log(stationIntake()));
        // stationIntaking.and(backwardMode).whileTrue(log(reverse(stationIntake())));
        // stationExtendedIntake.and(backwardMode.not()).whileTrue(log(stationExtendedIntake()));
        // stationExtendedIntake.and(backwardMode).whileTrue(log(reverse(stationExtendedIntake())));

        // scoreState.and(L2Coral).and(backwardMode.not()).onTrue(log(score2()));
        // scoreState.and(L2Coral).and(backwardMode).onTrue(log(reverse(score2())));
        // scoreState.and(L3Coral).and(backwardMode.not()).onTrue(log(score3()));
        // scoreState.and(L3Coral).and(backwardMode).onTrue(log(reverse(score3())));
        // scoreState.and(L4Coral).and(backwardMode.not()).onTrue(log(score4()));
        // scoreState.and(L4Coral).and(backwardMode).onTrue(log(reverse(score4())));

        // L2Algae.and(backwardMode.not()).whileTrue(log(l2Algae()));
        // L2Algae.and(backwardMode).whileTrue(log(reverse(l2Algae())));
        // L3Algae.and(backwardMode.not()).whileTrue(log(l3Algae()));
        // L3Algae.and(backwardMode).whileTrue(log(reverse(l3Algae())));

        // L1Coral.and(backwardMode.not()).whileTrue(log(l1Coral()));
        // L1Coral.and(backwardMode).whileTrue(log(reverse(l1Coral())));
        // L2Coral.and(backwardMode.not()).whileTrue(log(l2Coral()));
        // L2Coral.and(backwardMode).whileTrue(log(reverse(l2Coral())));
        // L3Coral.and(backwardMode.not()).whileTrue(log(l3Coral()));
        // L3Coral.and(backwardMode).whileTrue(log(reverse(l3Coral())));
        // L4Coral.and(backwardMode.not()).whileTrue(log(l4Coral()));
        // L4Coral.and(backwardMode).whileTrue(log(reverse(l4Coral())));

        // barge.and(backwardMode.not()).whileTrue(log(barge()));
        // barge.and(backwardMode).whileTrue(log(reverse(barge())));
        // homeAll.whileTrue(log(home()));

        // algaeHandoff.whileTrue(log(handOffAlgae()));
        // coralHandoff.whileTrue(log(handOffAlgae()));

        // TODO: for testing
        Robot.getPilot().testTune_tA.whileTrue(elbow.moveToMotorPosition(() -> 0.179));
        // Robot.getPilot().testTune_tA.whileTrue(elbow.moveToDegrees(config::getStationIntake));
        Robot.getPilot().testTune_tB.whileTrue(elbow.moveToDegrees(config::getL2Coral));
        Robot.getPilot().testTune_tX.whileTrue(elbow.moveToDegrees(config::getHome));
        Robot.getPilot().testTune_tY.whileTrue(score2());
        Robot.getPilot().reZero_start.onTrue(elbow.resetToInitialPos());
        // Robot.getPilot()
        //         .testTriggersTrigger
        //         .whileTrue(runElbow(() -> Robot.getPilot().getTestTriggersAxis()));
    }

    public static DoubleSupplier getPosition() {
        return () -> elbow.getPositionPercentage();
    }

    public static Command score2() {
        double newPos = config.getL2Coral() - 15;
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score2");
    }

    public static Command score3() {
        double newPos = 15 + config.getL3Coral();
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score3");
    }

    public static Command score4() {
        double newPos = 20 + config.getL4Coral();
        return elbow.moveToDegreesAndCheckReversed(() -> newPos).withName("Elbow.score4");
    }

    public static Command runElbow(DoubleSupplier speed) {
        return elbow.runPercentage(speed).withName("Elbow.runElbow");
    }

    public static Command home() {
        return elbow.moveToDegrees(config::getHome).withName("Elbow.home");
    }

    public static Command handOffAlgae() {
        return elbow.moveToDegrees(config::getHandAlgae).withName("Elbow.handOffAlgae");
    }

    /* Scoring positions */
    public static Command l2Algae() {
        return elbow.moveToDegreesAndCheckReversed(config::getL2Algae).withName("Elbow.l2Algae");
    }

    public static Command l3Algae() {
        return elbow.moveToDegreesAndCheckReversed(config::getL3Algae).withName("Elbow.l3Algae");
    }

    public static Command barge() {
        return elbow.moveToDegreesAndCheckReversed(config::getBarge).withName("Elbow.barge");
    }

    public static Command l1Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL1Coral).withName("Twist.L1Coral");
    }

    public static Command l2Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL2Coral).withName("Elbow.l2Coral");
    }

    public static Command l3Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL3Coral).withName("Elbow.l3Coral");
    }

    public static Command l4Coral() {
        return elbow.moveToDegreesAndCheckReversed(config::getL4Coral).withName("Elbow.l4Coral");
    }

    // missing auton Elbow commands, add when auton is added

    public static Command floorIntake() {
        return elbow.moveToDegrees(config::getFloorIntake).withName("Elbow.FloorIntake");
    }

    public static Command stationIntake() {
        return elbow.moveToDegreesAndCheckReversed(config::getStationIntake)
                .withName("Elbow.StationIntake");
    }

    public static Command stationExtendedIntake() {
        return elbow.moveToDegreesAndCheckReversed(config::getStationExtendedIntake)
                .withName("Shoulder.stationExtendedIntake");
    }

    public static Command coastMode() {
        return elbow.coastMode().withName("Elbow.CoastMode");
    }

    public static Command stopMotor() {
        return elbow.runStop().withName("Elbow.stop");
    }

    public static Command ensureBrakeMode() {
        return elbow.ensureBrakeMode().withName("Elbow.BrakeMode");
    }

    // Tune value command
    public static Command tuneElbow() {
        // return elbow.moveToDegrees(new TuneValue("Tune Elbow", 0).getSupplier())
        //         .withName("Elbow.Tune");
        return elbow.moveToDegrees(config::getTuneElbow);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }

    // Negate position command
    protected static Command reverse(Command cmd) {
        return cmd.deadlineFor(
                Commands.startEnd(() -> config.setReversed(true), () -> config.setReversed(false)));
    }
}
