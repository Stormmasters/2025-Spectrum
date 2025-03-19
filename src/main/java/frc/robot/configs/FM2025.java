package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2025 extends Config {

    public FM2025() {
        super();
        swerve.configEncoderOffsets(-0.393311, 0.111084, 0.053223, 0.386963);
        // swerve.configEncoderOffsets(-0.395264, 0.11499, 0.061279, 0.385742);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(false);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        twist.setAttached(false);
        intake.setAttached(false);
        climb.setAttached(false);

        shoulder.setCANcoderAttached(true);
        shoulder.setCANcoderOffset(-0.89917 + 0.20833333333333333333);
        elbow.setCANcoderAttached(true);
        elbow.setCANcoderOffset(1.066406);
    }
}
