package frc.spectrumLib.mechanism;

import frc.robot.RobotTelemetry;

public class MechanismCommands {

    protected MechanismCommands() {};

    public void setupDefaultCommand() {
        RobotTelemetry.print("Mech Command Default");
    };

    public void bindTriggers() {};
}
