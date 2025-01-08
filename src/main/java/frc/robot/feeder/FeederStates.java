package frc.robot.feeder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.amptrap.AmpTrapStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.feeder.Feeder.FeederConfig;
import frc.robot.operator.Operator;
import frc.spectrumLib.Telemetry;

public class FeederStates {
    private static Feeder feeder = Robot.getFeeder();
    private static FeederConfig config = Robot.getConfig().feeder;
    private static Operator operator = Robot.getOperator();

    // public static final Trigger hasNote = feeder.lasercan.lessThan(300);
    // public static final Trigger noNote = hasNote.not();

    public static void setupDefaultCommand() {
        feeder.setDefaultCommand(
                log(feeder.runStop().ignoringDisable(true).withName("Feeder.default")));
    }

    public static void setStates() {
        intaking.or(operator.feederFwd_Y).whileTrue(log(intake()));
        ejecting.or(operator.feederRev_fY).whileTrue(log(eject()));
        score.whileTrue(log(score()));
        // noteToAmp.whileTrue(log(feedToAmp()));

        // Feed a note to the amp trap if we are climbing and the elevator hasn't gone up
        climbRoutine.and(ElevatorStates.isHome, AmpTrapStates.noNote).whileTrue(log(feedToAmp()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    private static Command eject() {
        return feeder.runVelocity(config::getEject).withName("Feeder.eject");
    }

    private static Command score() {
        return feeder.runVelocity(config::getScore).withName("Feeder.score");
    }

    private static Command intake() {
        return feeder.runVelocity(config::getIntake).withName("Feeder.intake");
    }

    private static Command feedToAmp() {
        return feeder.runVelocity(config::getFeedToAmp).withName("Feeder.feedToAmp");
    }

    private static Command coastMode() {
        return feeder.coastMode();
    }

    private static Command ensureBrakeMode() {
        return feeder.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
