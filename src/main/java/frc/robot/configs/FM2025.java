package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2025 extends Config {

    public FM2025() {
        super();
        swerve.configEncoderOffsets(-0.395264, 0.11499, 0.061279, 0.385742);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(false);
        shoulder.setAttached(false);
        elbow.setAttached(false);
        twist.setAttached(false);
        intake.setAttached(false);
        climb.setAttached(false);
        groundIntake.setAttached(false);
    }
}
