package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();
        // TODO: config swerve encoder offsets

        // Attached Mechanisms
        pilot.setAttached(false);
        photonPilot.setAttached(true);
        operator.setAttached(false);
        photonOperator.setAttached(true);
        groundIntake.setAttached(true);
        coralIntake.setAttached(true);
        inClimb.setAttached(true);
        photonShoulder.setAttached(true);
        elbow.setAttached(false);
        twist.setAttached(false);
        elevator.setAttached(true); 
    }
    
}
