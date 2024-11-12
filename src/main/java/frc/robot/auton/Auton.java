package frc.robot.auton;

import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.RobotTelemetry;

public class Auton {
    private static final SendableChooser<Command> autonChooser = new SendableChooser<>();
    private static boolean autoMessagePrinted = true;
    private static double autonStart = 0;

    /**
     * Sets up the autonomous mode selectors for the robot.
     *
     * <p>This method configures the available autonomous routines that can be selected from the
     * SmartDashboard. It adds several options to the autonChooser, including: - "Basic Front 4":
     * Runs the "Basic Front 4" autonomous routine. - "Madtown": Runs the "Madtown" autonomous
     * routine. - "Do Nothing": Sets the default option to print "Do Nothing Auto ran".
     *
     * <p>The selected autonomous routine can be chosen from the SmartDashboard interface.
     */
    public void setupSelectors() {
        // Config Autos (Uncomment to use)
        // autonChooser.addOption("1 Meter", new PathPlannerAuto("1 Meter Auto")); // Runs full Auto
        // autonChooser.addOption("3 Meter", new PathPlannerAuto("3 Meter Auto")); // Runs full Auto

        autonChooser.addOption("Basic Front 4", SpectrumAuton("Basic Front 4")); // Runs full Auto
        autonChooser.addOption("Madtown", SpectrumAuton("Madtown")); // Runs full Auto
        autonChooser.setDefaultOption(
                "Do Nothing", Commands.print("Do Nothing Auto ran")); // Runs full Auto

        SmartDashboard.putData("Auto Chooser", autonChooser);
    }

    public Auton() {
        setupSelectors(); // runs the command to start the chooser for auto on shuffleboard
        RobotTelemetry.print("Auton Subsystem Initialized: ");
    }

    /**
     * Creates a SpectrumAuton command sequence.
     *
     * <p>This method generates a command sequence that first waits for 0.01 seconds and then
     * executes a PathPlannerAuto command with the specified autonomous routine name.
     *
     * @param autoName the name of the autonomous routine to execute
     * @return a Command that represents the SpectrumAuton sequence
     */
    public Command SpectrumAuton(String autoName) {
        return Commands.waitSeconds(0.01).andThen(new PathPlannerAuto(autoName));
    }

    /**
     * Retrieves the autonomous command selected on the shuffleboard.
     *
     * @return the selected autonomous command if one is chosen; otherwise, returns a PrintCommand
     *     indicating that the autonomous command is null.
     */
    public Command getAutonomousCommand() {
        Command auton = autonChooser.getSelected(); // sees what auto is chosen on shuffleboard
        if (auton != null) {
            return auton; // checks to make sure there is an auto and if there is it runs an auto
        } else {
            return new PrintCommand(
                    "*** AUTON COMMAND IS NULL ***"); // runs if there is no auto chosen, which
            // shouldn't happen because of the default
            // auto set to nothing which still runs
            // something
        }
    }

    /** This method is called in AutonInit */
    public void startAutonTimer() {
        autonStart = Timer.getFPGATimestamp();
        autoMessagePrinted = false;
    }

    /** Called in RobotPeriodic and displays the duration of the auton command Based on 6328 code */
    public void printAutoDuration() {
        Command autoCommand = getAutonomousCommand();
        if (autoCommand != null) {
            if (!autoCommand.isScheduled() && !autoMessagePrinted) {
                if (DriverStation.isAutonomousEnabled()) {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton finished in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                } else {
                    RobotTelemetry.print(
                            String.format(
                                    "*** Auton CANCELLED in %.2f secs ***",
                                    Timer.getFPGATimestamp() - autonStart));
                }
                autoMessagePrinted = true;
            }
        }
    }
}
