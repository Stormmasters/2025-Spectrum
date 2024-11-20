package frc.robot.feeder;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.amptrap.AmpTrapStates;
import frc.robot.elevator.ElevatorStates;
import frc.robot.feeder.Feeder.FeederConfig;
import frc.robot.operator.Operator;

public class FeederStates {
    private static Feeder feeder = Robot.getFeeder();
    private static FeederConfig config = Robot.getConfig().feeder;
    private static Operator operator = Robot.getOperator();

    public static void setupDefaultCommand() {
        feeder.setDefaultCommand(feeder.runStop().ignoringDisable(true).withName("Feeder.default"));
    }

    public static void setStates() {
        intaking.or(operator.feederFwd_Y).whileTrue(intake());
        ejecting.or(operator.feederRev_fY).whileTrue(eject());
        score.whileTrue(score());
        noteToAmp.whileTrue(feedToAmp());

        // Feed a note to the amp trap if we are climbing and the elevator hasn't gone up
        climbRoutine.and(ElevatorStates.isHome, AmpTrapStates.noNote).whileTrue(feedToAmp());

        coastMode.whileTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
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
}
