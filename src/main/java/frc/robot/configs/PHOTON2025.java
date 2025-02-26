package frc.robot.configs;

import frc.robot.Robot.Config;
import frc.robot.elevator.PhotonElevator;
import frc.robot.shoulder.PhotonShoulder;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();
        // TODO: config swerve encoder offsets

        elevator = new PhotonElevator.PhotonElevatorConfig();
        shoulder = new PhotonShoulder.PhotonShoulderConfig();

        // Attached Mechanisms
        pilot.setAttached(false);
        photonPilot.setAttached(true);
        operator.setAttached(false);
        photonOperator.setAttached(true);
        groundIntake.setAttached(false);
        coralIntake.setAttached(true);
        inClimb.setAttached(false);
        photonShoulder.setAttached(true);
        shoulder.setAttached(false);
        elbow.setAttached(false);
        twist.setAttached(false);
        elevator.setAttached(true);
    }
}
