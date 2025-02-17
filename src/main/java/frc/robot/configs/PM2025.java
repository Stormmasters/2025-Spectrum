package frc.robot.configs;

import frc.robot.Robot.Config;

public class PM2025 extends Config {

    public PM2025() {
        super();
        // swerve.configEncoderOffsets(0.017822, -0.411865, -0.290039, -0.424072);
        swerve.configEncoderOffsets(-0.018066, -0.099121, 0.289795, -0.098389);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);

        elevator.setAttached(false);
        shoulder.setAttached(false);
        elbow.setAttached(false);
        twist.setAttached(false);
        coralIntake.setAttached(false);

        inClimb.setAttached(false);
        groundIntake.setAttached(false);
    }
}
