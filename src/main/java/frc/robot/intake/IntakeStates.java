package frc.robot.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.intake.Intake.IntakeConfig;
import frc.robot.pilot.Pilot;

public class IntakeStates {
    private static Intake intake;
    private static IntakeConfig config;
    private static Pilot pilot = Robot.getPilot();

    public static void setupDefaultCommand() {
        intake.setDefaultCommand(intake.runStop().ignoringDisable(true).withName("Intake.default"));
    }

    public static void bindTriggers() {}

    public static Command runFull() {
        return intake.runVelocityTCFOCrpm(config::getMaxSpeed)
                .withName("Intake.runFull");
    }

    public static Command intake() {
        return intake.runVelocityTCFOCrpm(config::getIntake).withName("Intake.intake");
    }

    public static Command slowIntake() {
        return intake.runVelocityTCFOCrpm(config::getSlowIntake)
                .withName("Intake.slowIntake");
    }

    public static Command eject() {
        return intake.runVelocityTCFOCrpm(config::getEject).withName("Intake.eject");
    }

    public static Command coastMode() {
        return intake.coastMode();
    }

    public static Command intakeWithoutCurrentLimit() {
        return intake.intakeWithoutCurrentLimit();
    }

    public static Command stopMotor() {
        return intake.runStop().withName("Intake.stopMotor");
    }

    public static Command ensureBrakeMode() {
        return intake.ensureBrakeMode();
    }
}
