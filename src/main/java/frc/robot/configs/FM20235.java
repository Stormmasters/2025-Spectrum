package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM20235 extends Config{

    //2023.5 Off-season Robot

    public FM20235() {
        super();
        swerve.configEncoderOffsets(323.877, 67.4121, 136.49414, 7.998);

        // Attached Mechanisms
        pilot.setAttached(true);
        operator.setAttached(true);
        intake.setAttached(true);

        ampTrap.setAttached(false);
        climber.setAttached(false);
        elevator.setAttached(false);
        feeder.setAttached(false);
        launcher.setAttached(false);
        pivot.setAttached(false);
    }
}
