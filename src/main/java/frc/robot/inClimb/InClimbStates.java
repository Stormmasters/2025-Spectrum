package frc.robot.inClimb;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.inClimb.InClimb.InClimbConfig;
import frc.spectrumLib.Telemetry;
import java.util.function.DoubleSupplier;

public class InClimbStates {
    private static InClimb inClimb = Robot.getInClimb();
    private static InClimbConfig config = Robot.getConfig().inClimb;

    public static void setupDefaultCommand() {
        inClimb.setDefaultCommand(
                log(inClimb.runHoldInClimb().ignoringDisable(true).withName("InClimb.default")));
    }

    public static void setStates() {
        coastMode.onTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));

        groundAlgae.whileTrue(log(algaeFloorIntake()));
        groundAlgae.onFalse(log(home()));
        groundCoral.whileTrue(log(coralFloorIntake()));
        groundCoral.onFalse(log(home()));

        climbPrep.whileTrue(log(climbPrep()));
        climbFinish.whileTrue(log(climbFinish()));
        homeAll.onTrue(log(home()));
        processorLollipopScore.whileTrue(log(processorLollipopScore()));
        processorLollipopScore.onFalse(log(home()));

        homeInClimb.whileTrue(log(zero()));
    }

    public static Command runInClimb(DoubleSupplier speed) {
        return inClimb.runPercentage(speed).withName("InClimb.runInClimb");
    }

    // missing auton InClimb commands, add when auton is added

    public static Command zero() {
        return inClimb.zeroInClimbRoutine().withName("InClimb.zero");
    }

    public static Command intake() {
        return inClimb.moveToDegrees(config::getIntake).withName("InClimb.intake");
    }

    public static Command home() {
        return inClimb.moveToDegrees(config::getHome).withName("InClimb.home");
    }

    public static Command climbPrep() {
        return inClimb.moveToDegrees(config::getPrepClimber).withName("InClimb.prepClimber");
    }

    public static Command climbFinish() {
        return inClimb.moveToDegrees(config::getFinishClimb).withName("InClimb.finishClimb");
    }

    public static Command algaeFloorIntake() {
        return inClimb.moveToDegrees(config::getAlgaeFloorIntake)
                .withName("InClimb.algaeFloorIntake");
    }

    public static Command coralFloorIntake() {
        return inClimb.moveToDegrees(config::getCoralFloorIntake)
                .withName("InClimb.coralFloorIntake");
    }

    public static Command processorLollipopScore() {
        return inClimb.moveToDegrees(config::getProcessorScore).withName("InClimb.processorScore");
    }

    public static DoubleSupplier getPosition() {
        return (() -> inClimb.getPositionPercentage());
    }

    public static Command coastMode() {
        return inClimb.coastMode().withName("InClimb.CoastMode");
    }

    public static Command stopMotor() {
        return inClimb.runStop().withName("InClimb.stop");
    }

    public static Command ensureBrakeMode() {
        return inClimb.ensureBrakeMode().withName("InClimb.BrakeMode");
    }

    // Tune value command
    public static Command tuneInClimb() {
        return inClimb.moveToDegrees(config::getTuneInClimb);
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
