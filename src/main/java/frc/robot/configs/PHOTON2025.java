package frc.robot.configs;

import frc.robot.Robot.Config;

public class PHOTON2025 extends Config {
    public PHOTON2025() {
        super();
        // TODO: config swerve encoder offsets

        // Attached Mechanisms
        pilot.setAttached(true); // TODO: dettach and change to photonPilot
        operator.setAttached(true); // TODO: dettach and change to photonOperator
        groundIntake.setAttached(true);
        coralIntake.setAttached(true);
        inClimb.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(false);
        twist.setAttached(false);
        elevator.setAttached(true); 
    }
    
}
