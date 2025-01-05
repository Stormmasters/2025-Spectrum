package frc.robot.pivot;

import static frc.robot.RobotStates.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.pivot.Pivot.PivotConfig;
import frc.spectrumLib.Telemetry;
import frc.spectrumLib.TuneValue;
import java.util.function.DoubleSupplier;

public class PivotStates {
    private static Pivot pivot = Robot.getPivot();
    private static PivotConfig config = Robot.getConfig().pivot;

    public static void setupDefaultCommand() {
        pivot.setDefaultCommand(
                log(pivot.runHoldPivot().ignoringDisable(true).withName("Pivot.default")));
    }

    public static void setStates() {
        // missing bindTriggers
        intaking.whileTrue(log(subwoofer()));
        ampPrep.whileTrue(log(home()));

        coastMode.whileTrue(log(coastMode()));
        coastMode.onFalse(log(ensureBrakeMode()));
    }

    // missing distance based pivot commands

    public static Command runPivot(DoubleSupplier speed) {
        return pivot.runPercentage(speed).withName("Pivot.runPivot");
    }

    public static Command home() {
        return pivot.moveToPercentage(config::getHome).withName("Pivot.home");
    }

    public static Command climbHome() {
        return pivot.moveToPercentage(config::getClimbHome).withName("Pivot.climbHome");
    }

    public static Command manualFeed() {
        return pivot.moveToPercentage(config::getManualFeed).withName("Pivot.manualFeed");
    }

    /* Scoring positions */

    public static Command subwoofer() {
        return pivot.moveToPercentage(config::getSubwoofer).withName("Pivot.subwoofer");
    }

    public static Command podium() {
        return pivot.moveToPercentage(config::getPodium).withName("Pivot.podium");
    }

    public static Command ampWing() {
        return pivot.moveToPercentage(config::getAmpWing).withName("Pivot.ampWing");
    }

    public static Command fromAmp() {
        return pivot.moveToPercentage(config::getFromAmp).withName("Pivot.fromAmp");
    }

    public static Command intoAmp() {
        return pivot.moveToPercentage(config::getIntoAmp).withName("Pivot.intoAmp");
    }

    // missing auton pivot commands, add when auton is added

    public static Command intake() {
        return pivot.moveToPercentage(config::getIntake).withName("Pivot.intake");
    }

    public static Command coastMode() {
        return pivot.coastMode().withName("Pivot.CoastMode");
    }

    public static Command stopMotor() {
        return pivot.runStop().withName("Pivot.stop");
    }

    public static Command ensureBrakeMode() {
        return pivot.ensureBrakeMode().withName("Pivot.BrakeMode");
    }

    /** increase vision shots by 0.5 percent */
    public static Command increaseOffset() {
        return pivot.runOnce(pivot::increaseOffset)
                .withName("Pivot.increaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** decrease vision shots by 0.5 percent */
    public static Command decreaseOffset() {
        return pivot.runOnce(pivot::decreaseOffset)
                .withName("Pivot.decreaseFudgeFactor")
                .ignoringDisable(true);
    }

    /** reset fudge factor to 0 */
    public static Command resetOffset() {
        return pivot.runOnce(pivot::resetOffset)
                .withName("Pivot.resetFudgeFactor")
                .ignoringDisable(true);
    }

    public static Command switchFeedSpot() {
        return pivot.runOnce(pivot::switchFeedSpot)
                .withName("Pivot.switchFeedSpot")
                .ignoringDisable(true);
    }

    // Tune value command
    public static Command tunePivot() {
        return pivot.moveToPercentage(new TuneValue("Tune Pivot", 0).getSupplier())
                .withName("Pivot.Tune");
    }

    // Log Command
    protected static Command log(Command cmd) {
        return Telemetry.log(cmd);
    }
}
