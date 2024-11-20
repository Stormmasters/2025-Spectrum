package frc.robot.configs;

import edu.wpi.first.wpilibj.util.Color;

public class PM2024 extends FM2024 {

    public PM2024() {
        super();
        swerve.configEncoderOffsets(0.336426, -0.031006, -0.323730, 0.492188);
        launcher.setAttached(false);
        pivot.setAttached(false);
        leds.setPrimaryColor(Color.kPink);
    }
}
