package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2025 extends Config {

    public PM2025() {
        super();
        swerve.configEncoderOffsets(-0.399170, -0.189209, 0.121826, -0.021973);

        //         FL = 0.010254
        // BL = -0.290039
        // FR = -0.411865 rotations
        // BR = -0.424072 rotations
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
