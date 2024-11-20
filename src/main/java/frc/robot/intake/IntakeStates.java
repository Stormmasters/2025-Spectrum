package frc.robot.intake;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;

public class IntakeStates {
    private static Intake intake = Robot.getIntake();
    private static IntakeConfig config = Robot.getConfig().intake;

    public static final Trigger hasNote = intake.hasNote;
    public static final Trigger noNote = hasNote.not();

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(intake.runStop().ignoringDisable(true).withName("Intake.default"));
    }

    public static void setStates() {
        intaking.whileTrue(intake());
        ejecting.whileTrue(eject());
        score.whileTrue(intake());

        coastMode.whileTrue(coastMode());
        coastMode.onFalse(ensureBrakeMode());
    }

    private static Command intake() {
        return intake.runVelocityTcFocRpm(config::getIntake).withName("Intake.intake");
    }

    private static Command eject() {
        return intake.runVelocityTcFocRpm(config::getEject).withName("Intake.eject");
    }

    private static Command coastMode() {
        return intake.coastMode();
    }

    private static Command ensureBrakeMode() {
        return intake.ensureBrakeMode();
    }
}
