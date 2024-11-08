package frc.robot;

import frc.spectrumLib.mechanism.MechanismCommands;
import java.util.ArrayList;

/**
 * This class is used for commands that use multiple subsystems and don't directly call a gamepad.
 * This is often command groups such as moving an arm and turning on an intake, etc. In 2023 we
 * called this MechanismCommands.java
 */
public class RobotCommands {

    public static void setupRobotTriggers() {}

    protected static ArrayList<MechanismCommands> mechanismCommands =
            new ArrayList<MechanismCommands>();

    public static void addMechanismCommand(MechanismCommands mechanismCommand) {
        mechanismCommands.add(mechanismCommand);
    }

    public static void setupDefaultCommands() {
        for (MechanismCommands mechanismCommand : mechanismCommands) {
            mechanismCommand.setupDefaultCommand();
        }
    }

    public static void bindTriggers() {

        RobotTelemetry.print("Binding Triggers Robot Commands 2: " + mechanismCommands.size());
        for (MechanismCommands mechanismCommand : mechanismCommands) {
            mechanismCommand.bindTriggers();
            RobotTelemetry.print("Binding Triggers Robot Commands");
        }
    }
}
