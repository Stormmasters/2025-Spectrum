package frc.robot.climber;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.climber.Climber.ClimberConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class ClimberStates {
    private static Climber climber = Robot.getClimber();
    private static ClimberConfig config = Robot.getConfig().climber;

    public static void setupDefaultCommand() {
        climber.setDefaultCommand(
                log(climber.runHoldClimber().ignoringDisable(true).withName("climber.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
        climbPrep.whileTrue(log(prepClimber()));
        climbFinish.whileTrue(log(finishClimb()));
        homeAll.whileTrue(log(home()));
    }

    public static Command runClimber(DoubleSupplier speed) {
        return climber.runPercentage(speed).withName("Climber.runClimber");
    }

    public static Command prepClimber() {
        return climber.moveToPercentage(config::getPrepClimber).withName("Climber.prepClimber");
    }

    public static Command finishClimb() {
        return climber.moveToPercentage(config::getFinishClimb).withName("Climber.finishClimb");
    }

    // missing auton Climber commands, add when auton is added

    public static Command home() {
        return climber.moveToPercentage(config::getHome).withName("Climber.home");
    }

    public static Command coastMode() {
        return climber.coastMode().withName("Climber.CoastMode");
    }

    public static Command stopMotor() {
        return climber.runStop().withName("Climber.stop");
    }

    public static Command ensureBrakeMode() {
        return climber.ensureBrakeMode().withName("Climber.BrakeMode");
    }

    // Tune value command
    public static Command tuneClimber() {
        // return pivot.moveToPercentage(new TuneValue("Tune Climber", 0).getSupplier())
        //         .withName("Climber.Tune");
        return climber.moveToPercentage(config::getTuneClimber);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
