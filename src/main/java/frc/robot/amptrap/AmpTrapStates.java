package frc.robot.amptrap;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.amptrap.AmpTrap.AmpTrapConfig;
import frc.robot.climber.ClimberStates;
import frc.robot.elevator.ElevatorStates;
import frc.spectrumLib.Telemetry;

public class AmpTrapStates {
    private static AmpTrap ampTrap = Robot.getAmpTrap();
    private static AmpTrapConfig config = Robot.getConfig().ampTrap;

    public static final Trigger hasNote = ampTrap.hasNote;
    public static final Trigger noNote = hasNote.not();

    public static void setupDefaultCommand() {
        ampTrap.setDefaultCommand(
                log(ampTrap.runStop().ignoringDisable(true).withName("AmpTrap.default")));
    }

    public static void setStates() {
        intaking.whileTrue(log(intake()));
        ejecting.whileTrue(log(eject()));
        noteToAmp.and(noNote).whileTrue(log(amp()));
        score.whileTrue(log(score()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        // Trap once the climber is at the bottom
        climbRoutine.and(ElevatorStates.isHome, noNote).whileTrue(log(amp()));
        climbRoutine.and(ClimberStates.atHomePos).whileTrue(log(amp()));
    }

    private static Command intake() {
        return ampTrap.runVelocity(config::getIntake).withName("AmpTrap.intake");
    }

    private static Command amp() {
        return ampTrap.runVelocity(config::getAmp).withName("AmpTrap.amp");
    }

    private static Command score() {
        return ampTrap.runVelocity(config::getScore).withName("AmpTrap.score");
    }

    private static Command eject() {
        return ampTrap.runVelocity(config::getEject).withName("AmpTrap.eject");
    }

    private static Command coastMode() {
        return ampTrap.coastMode();
    }

    private static Command ensureBrakeMode() {
        return ampTrap.ensureBrakeMode();
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
