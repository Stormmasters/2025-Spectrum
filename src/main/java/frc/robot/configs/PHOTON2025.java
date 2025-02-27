package frc.robot.configs;

import frc.robot.Robot.Config;
import frc.robot.coralIntake.PhotonCoralIntakeConfig;
import frc.robot.elevator.PhotonElevatorConfig;
import frc.robot.shoulder.PhotonShoulderConfig;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();

        swerve.configEncoderOffsets(0.180908, -0.409912, -0.145508, 0.033691);

        elevator = new PhotonElevatorConfig();
        shoulder = new PhotonShoulderConfig();
        coralIntake = new PhotonCoralIntakeConfig();

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        elevator.setAttached(true);
        shoulder.setAttached(true);
        coralIntake.setAttached(true);
        inClimb.setAttached(false);
        groundIntake.setAttached(false);

        // Always false for Photon
        elbow.setAttached(false);
        twist.setAttached(false);
    }
}
