package frc.robot.configs;

import frc.robot.Robot.Config;
import frc.robot.coralIntake.PhotonCoralIntakeConfig;
import frc.robot.elevator.PhotonElevatorConfig;
import frc.robot.shoulder.PhotonShoulderConfig;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();

        swerve.configEncoderOffsets(0.490967, 0.410400, -0.208252, 0.422607);

        elevator = new PhotonElevatorConfig();
        shoulder = new PhotonShoulderConfig();
        coralIntake = new PhotonCoralIntakeConfig();

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(false);
        shoulder.setAttached(false);
        coralIntake.setAttached(false);
        inClimb.setAttached(true);
        groundIntake.setAttached(false);

        // Always false for Photon
        elbow.setAttached(false);
        twist.setAttached(false);
    }
}
