package frc.robot.configs;

import frc.robot.Robot.Config;
import frc.robot.elevator.PhotonElevatorConfig;
import frc.robot.shoulder.PhotonShoulderConfig;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();

        swerve.configEncoderOffsets(0.490967, 0.410400, -0.208252, 0.422607);

        elevator = new PhotonElevatorConfig();
        shoulder = new PhotonShoulderConfig();

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        groundIntake.setAttached(false);
        coralIntake.setAttached(true);
        inClimb.setAttached(false);
        shoulder.setAttached(true);
        elbow.setAttached(false);
        twist.setAttached(false);
        elevator.setAttached(true);
    }
}
