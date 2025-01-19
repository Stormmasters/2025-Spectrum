package frc.robot.configs;

import frc.robot.Robot.Config;

public class AM2025 extends Config {

    // Alpha Robot

    public AM2025() {
        super();
        swerve.configEncoderOffsets(-0.399170, -0.189209, 0.121826, -0.021973);

        // Attached Mechanisms
        //TODO: Add mechanism pivot based off elbow
        pilot.setAttached(true);
        operator.setAttached(true);
        algaeIntake.setAttached(true);
        coralIntake.setAttached(true);
        shoulder.setAttached(true);
        elbow.setAttached(true);
        wrist.setAttached(true);
        climber.setAttached(false);
        elevator.setAttached(true);
    }
}
