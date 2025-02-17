package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2025 extends Config {

    public PM2025() {
        super();
        swerve.configEncoderOffsets(0.010254, -0.411865, -0.290039, -0.424072);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        groundIntake.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        elevator.setAttached(true);
        coralIntake.setAttached(true);
        inClimb.setAttached(true);
    }
}
