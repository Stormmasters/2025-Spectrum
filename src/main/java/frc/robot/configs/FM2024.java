package frc.robot.configs;

import frc.robot.Robot.Config;

public class FM2024 extends Config {

    public FM2024() {
        super();
        swerve.configEncoderOffsets(0, 0, 0, 0);

        // Attached Mechanisms
        ampTrap.setAttached(true);
        climber.setAttached(true);
        elevator.setAttached(true);
        feeder.setAttached(true);
        intake.setAttached(true);
        launcher.setAttached(true);
        operator.setAttached(true);
        pivot.setAttached(true);
        pilot.setAttached(true);
        elbow.setAttached(false);
        shoulder.setAttached(false);

        // Pivot CANCoder configs
        /**
         * Flip sign of absolute position no offset. Enter value closer to 0 than recorded value.
         *
         * <p>Target 0.05 < motor position (rotations) < 0.15 when all the way down to keep shots
         * consistent
         *
         * <p>ex: absolute pos no offset = 0.87; pivotCANcoderOffset = -0.86; read motor position =
         * 0.13
         */
        pivot.configCANcoderOffset(-0.77);
    }
}
